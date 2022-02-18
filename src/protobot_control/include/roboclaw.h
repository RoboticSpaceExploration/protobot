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

#ifndef SRC_PROTOBOT_CONTROL_INCLUDE_ROBOCLAW_H_
#define SRC_PROTOBOT_CONTROL_INCLUDE_ROBOCLAW_H_

#include <ros/ros.h>
#include <stdint.h>
#include <termios.h>
#include "../include/settings.h"

class roboclaw {
 public:
    explicit roboclaw(settings* es_protobot);
    void SetupEncoders();
    void CloseEncoders();
    void SendCommandToWheels(double* cmd);
    void GetVelocityFromWheels(double* vel);

 private:
    int ClearIOBuffers();
    int WriteToEncoders(uint8_t* data, int nBytes);
    int WaitReadStatus(int nBytes, int timeout_ms);
    int ReadFromEncoders(int nBytes);
    int SendCommands(uint8_t* data, int writeBytes, int readBytes);
    uint8_t ScaleCommand(double cmd);
    double ConvertPulsesToRadians(double vel);
    uint32_t ValidateChecksum(uint8_t* packet, int nBytes);

    void ForwardM1(uint8_t address, uint8_t value);
    void ForwardM2(uint8_t address, uint8_t value);
    void BackwardM1(uint8_t address, uint8_t value);
    void BackwardM2(uint8_t address, uint8_t value);
    void ReadEncoderSpeedM1(uint8_t address);
    void ReadEncoderSpeedM2(uint8_t address);

    termios tty;
    settings* es;
    int serialPort;
    int zeroCmdVelCount;
    uint8_t buf[100];
};

#endif  // SRC_PROTOBOT_CONTROL_INCLUDE_ROBOCLAW_H_
