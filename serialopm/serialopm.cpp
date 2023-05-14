// BSD 3-Clause License
//
// Copyright (c) 2021, Aaron Giles
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>

#include "../src/ymfm.h"
#include "../src/ymfm_opm.h"

#define VERSION "0.5.0 (2023/05/14)"

using namespace std;
using namespace ymfm;

static bool g_abort_program = false;

void sigint_handler(int signal) {
  if (signal == SIGINT) {
    g_abort_program = true;
    std::cout << "CTRL+C is pressed." << std::endl;
  }
}

int32_t open_serial_port(const char* device_name) {

  int32_t rc = -1;

  int32_t port = open(device_name, O_RDWR);
  if (port <= 0) {
    std::cout << "Error: serial port open error." << std::endl;
    goto exit;
  }

  struct termios tty;
  if (tcgetattr(port, &tty) != 0) {
    std::cout << "Error: " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    goto exit;
  }

  cfmakeraw(&tty);

  tty.c_cc[VTIME] = 0; // non blocking mode
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, B38400);
  cfsetospeed(&tty, B38400);

  if (tcsetattr(port, TCSANOW, &tty) != 0) {
    std::cout << "Error: " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
    goto exit;
  }

  rc = port;

exit:
  return rc;
}

class SerialOPM {

  public:
    static const uint32_t OPM_CLOCK = 4000000;
    static const uint32_t OPM_SAMPLE_RATE = 62500;    // = 4000000 / 2 / 32
    static const uint32_t OPM_STEP = 16;              // = 1000000 / 62500;
    static const uint32_t OUT_SAMPLE_RATE = 44100;

  public:
    SerialOPM();
    ~SerialOPM();
    void reset();
    void generate();
    void write(uint8_t opm_reg, uint8_t opm_data);
    bool is_opm_register(uint8_t reg);
    bool is_busy();

  private:
    uint32_t downsample_counter;
    ymfm::ymfm_interface* ymfm_inf;
    ymfm::ym2151* opm_chip;

  public:
    uint32_t generated_counter;
    std::vector<uint8_t> out_pcm_data;

};

SerialOPM::SerialOPM() {

  this->ymfm_inf = new ymfm::ymfm_interface();
  this->opm_chip = new ymfm::ym2151(*(this->ymfm_inf));
  this->opm_chip->reset();

  this->reset();
}

SerialOPM::~SerialOPM() {
  delete this->opm_chip;
  delete this->ymfm_inf;
}

void SerialOPM::reset() {
  this->generated_counter = 0;
  this->downsample_counter = 0;
}

bool SerialOPM::is_opm_register(uint8_t reg) {
  return (reg == 0x01 || reg == 0x08 || reg == 0x0f || reg == 0x10 || reg == 0x11 ||
          reg == 0x12 || reg == 0x14 || reg == 0x18 || reg == 0x19 || reg == 0x1b ||
          (reg >= 0x20 && reg <= 0xff)) ? true : false;
}

void SerialOPM::generate() {

  ymfm::ym2151::output_data opm_pcm_data;
  this->opm_chip->generate(&opm_pcm_data);

  this->downsample_counter += SerialOPM::OUT_SAMPLE_RATE;
  if (this->downsample_counter >= SerialOPM::OPM_SAMPLE_RATE) {
    int16_t data_l = opm_pcm_data.data [ 0 ];   // output is 14bit PCM
    int16_t data_r = opm_pcm_data.data [ 1 ];
    this->out_pcm_data.push_back((((uint16_t)data_l) >> 8) & 0xff);
    this->out_pcm_data.push_back(((uint16_t)data_l) & 0xff);
    this->out_pcm_data.push_back((((uint16_t)data_r) >> 8) & 0xff);
    this->out_pcm_data.push_back(((uint16_t)data_r) & 0xff);
    this->downsample_counter -= SerialOPM::OPM_SAMPLE_RATE;
  }

  this->generated_counter++;
}

bool SerialOPM::is_busy() {
  return (this->opm_chip->read_status() & 0x80) ? true: false;
}

void SerialOPM::write(uint8_t opm_reg, uint8_t opm_data) {

  while (this->is_busy()) {
    this->generate();
  }
  this->opm_chip->write_address(opm_reg);

  while (this->is_busy()) {
    this->generate();
  }
  this->opm_chip->write_data(opm_data);

  this->generate();
}

int32_t main(int argc, char* argv[]) {

  // program exit code
  int32_t rc = -1;


  // work variables
  int32_t serial_port;
  static uint8_t read_buf [ 1024 ];
  memset(&read_buf, '\0', sizeof(read_buf));

  int32_t read_len = 0;

  uint8_t opm_reg = 0;
  uint8_t opm_data = 0;

  bool started = false;
  struct timeval start_time = { 0 };
  struct timeval current_time = { 0 };

  FILE* fp_out = NULL;


  // credit
  std::cout << "Serial OPM emulator - serialopm version " << VERSION << " tantan" << std::endl;
  std::cout << "--" << std::endl;
  std::cout << "ymfm Copyright (c) 2021, Aaron Giles" << std::endl;
  std::cout << "All rights reserved." << std::endl;
  std::cout << "--" << std::endl;


  // serial OPM engine
  SerialOPM opm = SerialOPM();
  std::cout << "Completed YM2151 engine initialization." << std::endl;


  // usage
  if (argc < 3) {
    std::cout << "usage: serialopm <serial-device> <output-file>" << std::endl;
    goto exit;
  }


  // UART receiver
  serial_port = open_serial_port(argv[1]);
  if (serial_port <= 0) {
    goto exit;
  }
  std::cout << "Completed serial port initialization." << std::endl;


  // sigint signal handler
  g_abort_program = false;
  signal(SIGINT, sigint_handler);


  // stream header detection
  read_len = 0;
  while (!g_abort_program) {

    int32_t len = read(serial_port, read_buf + read_len, 12 - read_len);
    if (len < 0) {
      printf("Error reading: %s", strerror(errno));
      rc = -2;
      break;
    }

    if (len == 0) continue;

    read_len += len;
    if (read_len < 12) continue;     // minimum length to detect 6 messages

    printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", 
            read_buf[0], read_buf[1], read_buf[2], read_buf[3], read_buf[4], read_buf[5],
            read_buf[6], read_buf[7], read_buf[8], read_buf[9], read_buf[10], read_buf[11]);

    if (opm.is_opm_register(read_buf[0]) && opm.is_opm_register(read_buf[2]) &&
        opm.is_opm_register(read_buf[4]) && opm.is_opm_register(read_buf[6]) &&
        opm.is_opm_register(read_buf[8]) && opm.is_opm_register(read_buf[10])) {
      opm.write(read_buf[0], read_buf[1]);
      opm.write(read_buf[2], read_buf[3]);
      opm.write(read_buf[4], read_buf[5]);
      opm.write(read_buf[6], read_buf[7]);
      opm.write(read_buf[8], read_buf[9]);
      opm.write(read_buf[10], read_buf[11]);
      std::cout << "Detected OPM data stream." << std::endl;
      break;
    } else {
      memmove(read_buf, read_buf + 1, 11);
      read_len = 9;
    }

  }


  // prep output file
  if (!g_abort_program) {
    fp_out = fopen(argv[2], "wb");
    gettimeofday(&start_time, NULL);
  }


  // main loop
  while (!g_abort_program) {

    int32_t len = read(serial_port, read_buf, sizeof(read_buf));
    if (len < 0) {
      printf("Error reading: %s", strerror(errno));
      rc = -2;
      break;
    }

    for (int32_t i = 0; i < len; i++) {
      if (opm_reg == 0) {
        opm_reg = read_buf[ i ];
      } else {
        opm_data = read_buf[ i ];
        opm.write(opm_reg, opm_data);
        opm_reg = 0;
        opm_data = 0;
      }
    }

    gettimeofday(&current_time, NULL);

    uint32_t elapsed_usec = (current_time.tv_sec - start_time.tv_sec) * 1000000 + (current_time.tv_usec - start_time.tv_usec);
    uint32_t total_ticks = elapsed_usec / SerialOPM::OPM_STEP;

    while (opm.generated_counter < total_ticks) {
      opm.generate();
    }

    if (opm.out_pcm_data.size() >= SerialOPM::OUT_SAMPLE_RATE * 4 * 1) {
      fwrite(opm.out_pcm_data.data(), 1, opm.out_pcm_data.size(), fp_out);
      opm.out_pcm_data.clear();
      printf(".");
      fflush(stdout);
    }

  }

  close(serial_port);

  if (fp_out != NULL) {

    if (opm.out_pcm_data.size() >= 4) {
      fwrite(opm.out_pcm_data.data(), 1, opm.out_pcm_data.size(), fp_out);
      opm.out_pcm_data.clear();
      printf(".");
      fflush(stdout);
    }

    fclose(fp_out);

  }

  if (rc == -1) rc = 0;

  std::cout << "Stopped." << std::endl;

exit:
  return rc;
}
