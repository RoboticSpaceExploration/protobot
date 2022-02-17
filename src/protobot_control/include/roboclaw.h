#ifndef SRC_PROTOBOT_CONTROL_ROBOCLAW_H_
#define SRC_PROTOBOT_CONTROL_ROBOCLAW_H_

#include<ros/ros.h>
#include <stdint.h>
#include <termios.h>
#include "settings.h"

class roboclaw {
public:
  roboclaw(settings* es_protobot);
    void SetupEncoders();                                                    // setup motor encoders
    void CloseEncoders();                                                    // close encoder connection
    void SendCommandToWheels(double* cmd);                                   // send cmd_vel to wheels
    void GetVelocityFromWheels(double* vel);                                 // get joint state status

private:
    int ClearIOBuffers();                                                    // flush I/O buffers
    int WriteToEncoders(uint8_t* data, int nBytes);                          // write to encoders
    int WaitReadStatus(int nBytes, int timeout_ms);                          // wait for encoders to fill reply data to buffer, returns 1 on success
    int ReadFromEncoders(int nBytes);                                        // read data from encoders buffer
    int SendCommands(uint8_t* data, int writeBytes, int readBytes);          // send specified commands to encoders
    uint8_t ScaleCommand(double cmd);
    double ConvertPulsesToRadians(double vel);
    uint32_t ValidateChecksum(uint8_t* packet, int nBytes);                  // get cyclic redundancy checksum (crc), used to validate

    void ForwardM1(uint8_t address, uint8_t value);                          // move M1 Motors forward
    void ForwardM2(uint8_t address, uint8_t value);                          // move M2 Motors forward
    void BackwardM1(uint8_t address, uint8_t value);                         // move M1 Motors backward
    void BackwardM2(uint8_t address, uint8_t value);                         // move M2 motors backward
    void ReadEncoderSpeedM1(uint8_t address);                                // read M1 encoder speed
    void ReadEncoderSpeedM2(uint8_t address);                                // read M2 encoder speed

    struct termios tty;                                                      // declare serial termios struct, used to configure serial port
    int serialPort;                                                          // roboclaw serial port file descriptor
    uint8_t buf[100];                                                        // temp buffer
    int zeroCmdVelCount;
    settings* es;
};

#endif  //SRC_PROTOBOT_CONTROL_ROBOCLAW_H_
