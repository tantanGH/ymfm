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

#define VERSION "0.4.0 (2023/05/13)"

#define OPM_SAMPLE_RATE     (62500)
#define OUT_SAMPLE_RATE     (44100)
#define OUT_PCM_BUFFER_SIZE (OUT_SAMPLE_RATE * 4 * 180)

volatile bool abort_program = false;

void signal_handler(int signal) {
  if (signal == SIGINT) {
    printf("CTRL+C is pressed.\n");
    abort_program = true;
  }
}

int32_t open_serial_port(const char* device_name) {

  int32_t rc = -1;

  int32_t serial_port = open(device_name, O_RDWR);
  if (serial_port <= 0) goto exit;

  struct termios tty;
  if (tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    goto exit;
  }

  cfmakeraw(&tty);

  tty.c_cc[VTIME] = 0; // non blocking mode
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, B38400);
  cfsetospeed(&tty, B38400);

  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    goto exit;
  }

  rc = serial_port;

exit:
  return rc;
}

bool is_opm_register(uint8_t reg) {
  return (reg == 0x01 || reg == 0x08 || reg == 0x0f || reg == 0x10 || reg == 0x11 ||
          reg == 0x12 || reg == 0x14 || reg == 0x18 || reg == 0x19 || reg == 0x1b ||
          (reg >= 0x20 && reg <= 0xff)) ? true : false;
}

void opm_generate(ymfm::ym2151& opm_chip, std::vector<uint8_t>& out_pcm_data, uint32_t& downsample_counter) {

  ymfm::ym2151::output_data opm_pcm_data;
  opm_chip.generate(&opm_pcm_data);

  downsample_counter += OUT_SAMPLE_RATE;
  if (downsample_counter >= OPM_SAMPLE_RATE) {
    int16_t data_l = opm_pcm_data.data [ 0 ];   // output is 14bit PCM
    int16_t data_r = opm_pcm_data.data [ 1 ];
    out_pcm_data.push_back((((uint16_t)data_l) >> 8) & 0xff);
    out_pcm_data.push_back(((uint16_t)data_l) & 0xff);
    out_pcm_data.push_back((((uint16_t)data_r) >> 8) & 0xff);
    out_pcm_data.push_back(((uint16_t)data_r) & 0xff);
    downsample_counter -= OPM_SAMPLE_RATE;
  }
}

uint32_t opm_write(ymfm::ym2151& opm_chip, std::vector<uint8_t>& out_pcm_data, uint32_t& downsample_counter, uint8_t opm_reg, uint8_t opm_data) {
  uint32_t opm_generated = 0;
  while (opm_chip.read_status() & 0x80) {
    opm_generate(opm_chip, out_pcm_data, downsample_counter);
    opm_generated++;
  }
  opm_chip.write_address(opm_reg);
  while (opm_chip.read_status() & 0x80) {
    opm_generate(opm_chip, out_pcm_data, downsample_counter);
    opm_generated++;
  }
  opm_chip.write_data(opm_data);
  return opm_generated;
}

int32_t main(int argc, char* argv[]) {

  int32_t rc = -1;

  static uint8_t read_buf [ 1024 ];
  memset(&read_buf, '\0', sizeof(read_buf));

  int32_t read_len = 0;

  uint8_t opm_reg = 0;
  uint8_t opm_data = 0;

  bool started = false;
  struct timeval start_time = { 0 };
  struct timeval current_time = { 0 };

  std::vector<uint8_t> out_pcm_data;
  uint32_t downsample_counter = 0;
  uint32_t out_pcm_index = 0;

  FILE* fp_out = NULL;

  // credit
  printf("Serial OPM emulator - serialopm version " VERSION " tantan\n");
  printf("--\n");
  printf("ymfm Copyright (c) 2021, Aaron Giles\n");
  printf("All rights reserved.\n");
  printf("--\n");


  // ym2151 engine
  ymfm::ymfm_interface ymfm_inf = ymfm::ymfm_interface();
  ymfm::ym2151 opm_chip = ymfm::ym2151(ymfm_inf);
  opm_chip.reset();

  uint32_t opm_clock = 4000000;                                   // X68000 is using 4MHz, not 3.58MHz
  uint32_t opm_sample_rate = opm_chip.sample_rate(opm_clock);     // 4000000/2/32 = 62500Hz
  uint32_t opm_step = 1000000 / opm_sample_rate;                  // 1000000/62500 = 16us
  uint32_t opm_generated = 0;
 
  uint32_t out_sample_rate = 44100;

  printf("Completed YM2151 engine initialization.\n");


  // UART receiver
//  int32_t serial_port = open_serial_port("/dev/serial0");
  int32_t serial_port = open_serial_port("/dev/tty.usbserial-A10ORWNF");
  if (serial_port <= 0) goto exit;
  printf("Completed serial port initialization.\n");


  // signal handler
  signal(SIGINT, signal_handler);

  // stream header detection
  read_len = 0;
  while (!abort_program) {

    int32_t len = read(serial_port, read_buf + read_len, 10 - read_len);
    if (len < 0) {
      printf("Error reading: %s", strerror(errno));
      rc = -2;
      break;
    }

    if (len == 0) continue;

    read_len += len;
    if (read_len < 10) continue;     // minimum length to detect 5 messages

    printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", 
            read_buf[0], read_buf[1], read_buf[2], read_buf[3], read_buf[4], 
            read_buf[5], read_buf[6], read_buf[7], read_buf[8], read_buf[9]);

    if (is_opm_register(read_buf[0]) &&
        is_opm_register(read_buf[2]) &&
        is_opm_register(read_buf[4]) &&
        is_opm_register(read_buf[6]) &&
        is_opm_register(read_buf[8])) {
      opm_write(opm_chip, out_pcm_data, downsample_counter, read_buf[0], read_buf[1]);
      opm_write(opm_chip, out_pcm_data, downsample_counter, read_buf[2], read_buf[3]);
      opm_write(opm_chip, out_pcm_data, downsample_counter, read_buf[4], read_buf[5]);
      opm_write(opm_chip, out_pcm_data, downsample_counter, read_buf[6], read_buf[7]);
      opm_write(opm_chip, out_pcm_data, downsample_counter, read_buf[8], read_buf[9]);
      break;
    } else {
      memmove(read_buf, read_buf + 1, 9);
      read_len = 9;
    }

  }

  // output file prep
  if (!abort_program) {
    fp_out = fopen("output.s44", "wb");
  }

  gettimeofday(&start_time, NULL);

  // main loop
  while (!abort_program) {

    int32_t len = read(serial_port, read_buf, sizeof(read_buf));
    if (len < 0) {
      printf("Error reading: %s", strerror(errno));
      rc = -2;
      break;
    }

    for (int32_t i = 0; i < len; i++) {
      if (opm_reg == 0) {
        opm_reg = read_buf[ i ];
        if (i + 1 < len) {
          opm_data = read_buf[ i + 1 ];
          i++;
          opm_generated += opm_write(opm_chip, out_pcm_data, downsample_counter, opm_reg, opm_data);
          opm_reg = 0;
          opm_data = 0;
        }
      } else {
        opm_data = read_buf[ i ];
        opm_generated += opm_write(opm_chip, out_pcm_data, downsample_counter, opm_reg, opm_data);
        opm_reg = 0;
        opm_data = 0;
      }
    }

    gettimeofday(&current_time, NULL);
    uint32_t elapsed_usec = (current_time.tv_sec - start_time.tv_sec) * 1000000 + (current_time.tv_usec - start_time.tv_usec);
    uint32_t total_ticks = elapsed_usec / opm_step;
    while (opm_generated < total_ticks) {
      opm_generate(opm_chip, out_pcm_data, downsample_counter);
      opm_generated++;
    }
    if (out_pcm_data.size() >= OUT_PCM_BUFFER_SIZE) {
      fwrite(out_pcm_data.data(), 1, out_pcm_data.size(), fp_out);
      out_pcm_data.clear();
      printf(".");
      fflush(stdout);
    }

  }

  close(serial_port);

  if (fp_out != NULL) {

    if (out_pcm_data.size() >= 4) {
      fwrite(out_pcm_data.data(), 1, out_pcm_data.size(), fp_out);
      out_pcm_data.clear();
      printf(".");
      fflush(stdout);
    }

    fclose(fp_out);

  }

  if (rc == -1) rc = 0;

  printf("Stopped.\n");

exit:
  return rc;
}
