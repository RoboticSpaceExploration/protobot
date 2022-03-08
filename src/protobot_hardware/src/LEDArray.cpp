/* MIT License

Copyright (c) [2022] [VIP Team RoSE]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>  // serial read() and write() defined here
#include <sys/time.h>
#include <assert.h>
#include <math.h>
#include <cstdlib>
#include <cstring>
#include "../include/LEDSettings.h"
#include "../include/LEDArray.h"

LEDArray::LEDArray() {
  for (int i = 0; i < 256; i++)
    errorBuf[i] = 0x00;
}

void LEDArray::GetBaudRate() {
  switch (ls->baudRate) {
    case 9600:
      baudRate = B9600;
      break;
    case 19200:
      baudRate = B19200;
      break;
    case 38400:
      baudRate = B38400;
      break;
    case 57600:
      baudRate = B57600;
      break;
    case 115200:
      baudRate = B115200;
      break;
    case 230400:
      baudRate = B230400;
      break;
    case 460800:
      baudRate = B460800;
      break;
    default:
      ROS_ERROR("Invalid Baud Rate Selection, setting to 115200");
      baudRate = B115200;
      break;
  }
}

void LEDArray::LEDInit(LEDSettings* ls_ptr) {
  ls = ls_ptr;
  // enable read & write, disable controlling terminal
  serialPort = open(ls->serialPortAddr.c_str(), O_RDWR | O_NOCTTY);

  if (serialPort < 0) {
    errorBufPtr = strerror_r(errno, errorBuf, sizeof(errorBuf));
    ROS_ERROR("LED Array: Could not open %s: Error %i from open: %s",
              ls->serialPortAddr.c_str(), errno, errorBufPtr);
    exit(EXIT_FAILURE);
  }

  fcntl(serialPort, F_SETFL, 0);  // set to blocking mode (for reads)

  if (tcgetattr(serialPort, &tty) != 0) {
    errorBufPtr = strerror_r(errno, errorBuf, sizeof(errorBuf));
    ROS_ERROR("LED Array: Error %i from tcgetattr: %s", errno, errorBufPtr);
    exit(EXIT_FAILURE);
  }

  // set necessary bits

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;

  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN]  = 0;

  // set baud rates

  cfsetispeed(&tty, baudRate);
  cfsetospeed(&tty, baudRate);

  // save flag settings

  if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
    strerror_r(errno, errorBuf, sizeof(errorBuf));
    ROS_ERROR("LED Array: Error %i from tcsetattr: %s", errno, errorBuf);
  }

  ClearIOBuffers();
}

void LEDArray::LEDQuit() {
  close(serialPort);
}

int LEDArray::ClearIOBuffers() {
  return tcflush(serialPort, TCIOFLUSH);
}

void LEDArray::ToggleLEDArray(int8_t flag) {
  ClearIOBuffers();
  int writeFlag;
  for (int i = 0; i < 4; i++) {
    writeFlag = write(serialPort, &flag, 1);
    usleep(250000);
  }
  ROS_INFO("Wrote [%d] byte(s) to LED Array: cmd: [%d]", writeFlag, flag);
  ClearIOBuffers();
}


