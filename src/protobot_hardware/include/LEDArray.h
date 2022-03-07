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

#ifndef SRC_PROTOBOT_HARDWARE_INCLUDE_LEDARRAY_H_
#define SRC_PROTOBOT_HARDWARE_INCLUDE_LEDARRAY_H_

#include <ros/ros.h>
#include <termios.h>
#include <stdint.h>
#include "../include/LEDSettings.h"

class LEDArray {
 public:
  explicit LEDArray(LEDSettings* ls_ptr);
  void LEDInit();
  void LEDQuit();
  void ToggleLEDArray(int8_t flag);

 private:
  int ClearIOBuffers();
  void GetBaudRate();
  termios tty;
  int serialPort;
  unsigned int baudRate;
  LEDSettings* ls;
  char* errorBufPtr;
  char errorBuf[256];
};

#endif  // SRC_PROTOBOT_HARDWARE_INCLUDE_LEDARRAY_H_
