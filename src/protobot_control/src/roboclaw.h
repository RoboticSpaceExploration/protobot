//
// Created by jacob on 2/7/22.
//

#ifndef PROTOBOT_ROBOCLAW_H
#define PROTOBOT_ROBOCLAW_H

#include<iostream>
#include<cstdlib>
#include<cstring>
#include<stdint.h>
#include<fcntl.h>
#include<errno.h>
#include<termios.h>
#include<unistd.h>
#include<sys/ioctl.h>
#include<sys/time.h>
#include<assert.h>
#include "protobot.h"

class roboclaw : public protobot {

public:
    roboclaw();
    void SetupEncoders();                                                    // setup motor encoders
    void CloseEncoders();                                                    // close encoder connection
    void Read();                                                             // inherited from protobot.h
    void Write();                                                            // inherited from protobot.h

private:
    int ClearIOBuffers();                                                    // flush I/O buffers
    int WriteToEncoders(uint8_t* data, int nBytes);                          // write to encoders
    int WaitReadStatus(int nBytes, int timeout_ms);                          // wait for encoders to fill reply data to buffer, returns 1 on success
    int ReadFromEncoders(int nBytes);                                        // read data from encoders buffer
    int SendCommands(uint8_t* data, int writeBytes, int readBytes);          // send specified commands to encoders
    uint32_t ValidateChecksum(uint8_t* packet, int nBytes);                  // get cyclic redundancy checksum (crc), used to validate

    void ForwardM1(uint8_t address, uint8_t value);                          // move M1 Motors
    void ForwardM2(uint8_t address, uint8_t value);                          // move M2 Motors
    void DriveForwardM1(uint8_t address, uint32_t speed);                    // move M1 motors signed speed
    void DriveForwardM2(uint8_t address, uint32_t speed);                    // move M2 motors signed speed
    void ReadEncoderSpeedM1(uint8_t address);                                // read M1 encoder speed
    void ReadEncoderSpeedM2(uint8_t address);                                // read M2 encoder speed

    struct termios tty;                                                      // declare serial termios struct, used to configure serial port
    int serialPort;                                                          // roboclaw serial port file descriptor
    uint8_t buf[100];                                                        // temp buffer

};

#endif //PROTOBOT_ROBOCLAW_H
