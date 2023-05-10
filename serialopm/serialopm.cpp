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

//using namespace std;

#define VERSION "0.3.0 (2023/05/10)"

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

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  tty.c_cc[VTIME] = 10;
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, B19200);
  cfsetospeed(&tty, B19200);

  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    goto exit;
  }

  rc = serial_port;

exit:
  return rc;
}

int32_t main(int argc, char* argv[]) {

  int32_t rc = -1;

  static uint8_t read_buf [ 1024 ];
  memset(&read_buf, '\0', sizeof(read_buf));

  int16_t opm_reg = -1;
  int16_t opm_data = -1;

  bool started = false;
  struct timeval start_time = { 0 };
  struct timeval current_time = { 0 };

  #define OPM_PCM_BUFFER_SIZE (62500*2)
  #define OUT_PCM_BUFFER_SIZE (16384)

  static uint8_t out_pcm_data[ OUT_PCM_BUFFER_SIZE ];
  uint32_t downsample_counter = 0;
  uint32_t out_pcm_index = 0;

  FILE* fp_out = NULL;
  
  // credit
  printf("Serial OPM emnulator - serialopm version " VERSION " tantan\n");
  printf("--\n");
  printf("ymfm Copyright (c) 2021, Aaron Giles\n");
  printf("All rights reserved.\n");
  printf("--\n");


  // ym2151 engine
  ymfm::ymfm_interface ymfm_inf = ymfm::ymfm_interface();
  ymfm::ym2151 opm_chip = ymfm::ym2151(ymfm_inf);
  ymfm::ym2151::output_data opm_pcm_data;
  opm_chip.reset();

  uint32_t opm_clock = 4000000;                                   // X68000 is using 4MHz, not 3.58MHz
  uint32_t opm_sample_rate = opm_chip.sample_rate(opm_clock);     // 4000000/2/32 = 62500Hz
 
  uint32_t out_sample_rate = 44100;
  uint32_t out_interval = 1000000 / opm_sample_rate;              // 1000000/62500 = 16us

  printf("Completed YM2151 engine initialization.\n");


  // UART receiver
//  int32_t serial_port = open_serial_port("/dev/serial0");
  int32_t serial_port = open_serial_port("/dev/tty.usbserial-A10ORWNF");
  if (serial_port <= 0) goto exit;
  printf("Completed serial port initialization.\n");


  // signal handler
  signal(SIGINT, signal_handler);


  // service preparation
  fp_out = fopen("output.s44", "wb");
 
  // main loop
  while (!abort_program) {

    int len = read(serial_port, read_buf, sizeof(read_buf));
    if (len < 0) {
      printf("Error reading: %s", strerror(errno));
      rc = -2;
      break;
    }
    if (len == 0) continue;
    //printf("%d bytes\n", len);
    
    for (int16_t i = 0; i < len; i++) {
      if (read_buf[ i ] == 0x17) {
        if (i + 1 < len) {
          opm_reg = read_buf[ i + 1 ];
          i++;
          if (i + 1 < len) {
            opm_data = read_buf[ i + 1 ];
            i++;
            while (opm_chip.read_status() & 0x80) {
              usleep(100);
            }
            opm_chip.write_address(opm_reg);
            while (opm_chip.read_status() & 0x80) {
              usleep(100);
            }
            opm_chip.write_data(opm_data);
            if (!started) {
              started = true;
              gettimeofday(&start_time, NULL);
            }
            opm_reg = -1;
            opm_data = -1;
            //printf(".");
          } else {
            opm_data = -1;
          }
        } else {
          opm_reg = -1;
        }
      } else if (opm_reg == -1) {
        opm_reg = read_buf[ i ];
        if (i + 1 < len) {
          opm_data = read_buf[ i + 1 ];
          i++;
          while (opm_chip.read_status() & 0x80) {
            usleep(100);
          }
          opm_chip.write_address(opm_reg);
          while (opm_chip.read_status() & 0x80) {
            usleep(100);
          }
          opm_chip.write_data(opm_data);
          if (!started) {
            gettimeofday(&start_time, NULL);
            started = true;
          }
          opm_reg = -1;
          opm_data = -1;
          //printf(".");
        }
      } else {
        opm_data = read_buf[ i ];
        while (opm_chip.read_status() & 0x80) {
          usleep(100);
        }
        opm_chip.write_address(opm_reg);
        while (opm_chip.read_status() & 0x80) {
          usleep(100);
        }
        opm_chip.write_data(opm_data);
        if (!started) {
          gettimeofday(&start_time, NULL);
          started = true;
        }
        opm_reg = -1;
        opm_data = -1;
        //printf(".");
      }
    }

    if (started) {

      gettimeofday(&current_time, NULL);
      uint32_t delay = (current_time.tv_sec - start_time.tv_sec) * 1000000 + (current_time.tv_usec - start_time.tv_usec);

      if (delay >= out_interval) {

          uint32_t num_samples = (uint32_t)( delay / out_interval );          
          for (uint32_t i = 0; i < num_samples; i++) {
            opm_chip.generate(&opm_pcm_data, 1);
            downsample_counter += out_sample_rate;
            if (downsample_counter >= opm_sample_rate) {
              int16_t data_l = opm_pcm_data.data [ 0 ] / 65536;   // output is 14bit PCM
              int16_t data_r = opm_pcm_data.data [ 1 ] / 65536;
              out_pcm_data [ out_pcm_index ++ ] = (((uint16_t)data_l) >> 8) & 0xff;
              out_pcm_data [ out_pcm_index ++ ] = ((uint16_t)data_l) & 0xff;
              out_pcm_data [ out_pcm_index ++ ] = (((uint16_t)data_r) >> 8) & 0xff;
              out_pcm_data [ out_pcm_index ++ ] = ((uint16_t)data_r) & 0xff;
              if (out_pcm_index >= OUT_PCM_BUFFER_SIZE) {
                fwrite(out_pcm_data, 1, out_pcm_index, fp_out);
                out_pcm_index = 0;
                printf(".");
                fflush(stdout);
              }
              downsample_counter -= opm_sample_rate;
            }
          }

          start_time = current_time;
      }

    }

  }

  close(serial_port);

  if (fp_out != NULL) {

    if (out_pcm_index >= 4) {
      fwrite(out_pcm_data, 1, out_pcm_index, fp_out);
      out_pcm_index = 0;
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
