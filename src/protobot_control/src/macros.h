//
// Created by jacob on 2/7/22.
//

#ifndef PROTOBOT_MACROS_H
#define PROTOBOT_MACROS_H

#define SERIAL_PORT  "/dev/ttyAMA0" // serial port location
#define BAUD_RATE    B115200        // roboclaw encoder baud rate
#define TIMEOUT_DS   1              // timeout period, 1 ds = 100 ms (not used at the moment)
#define TIMEOUT_MS   12             // Actual encoder timeout, according to this library: https://github.com/bmegli/roboclaw
#define RETRIES      3              // number of retries
#define M1FORWARD    0              // M1 Forward command
#define M2FORWARD    4              // M2 Forward command
#define M1BACKWARD   1              // M1 Backward command
#define M2BACKWARD   5              // M2 Backward command
#define M1DRIVE      35             // drive M1 with signed speed (not implemented)
#define M2DRIVE      36             // drive M2 with signed speed (not implemented)
#define M1_READ_ENCODER_SPEED 18    // read M1 encoder speed
#define M2_READ_ENCODER_SPEED 19    // read M2 encoder speed
#define ROBOCLAW_ACK_BYTES 1        // number of acknowledgement bytes
#define ROBOCLAW_ACK_BYTE  0xFF     // roboclaw acknowledgement byte upon successful command
#define MAX_BUF        100          // max buffer size read from encoders
#define MAX_M1M2_VALUE 127          // maximum allowable drive command

#endif //PROTOBOT_MACROS_H
