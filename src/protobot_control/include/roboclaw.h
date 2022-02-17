#ifndef SRC_PROTOBOT_CONTROL_INCLUDE_ROBOCLAW_H_
#define SRC_PROTOBOT_CONTROL_INCLUDE_ROBOCLAW_H_

#include<ros/ros.h>
#include <stdint.h>
#include <termios.h>
#include "settings.h"

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
    int serialPort;
    uint8_t buf[100];
    int zeroCmdVelCount;
    settings* es;
};

#endif  // SRC_PROTOBOT_CONTROL_INCLUDE_ROBOCLAW_H_
