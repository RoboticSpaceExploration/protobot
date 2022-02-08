//
// Created by jacob on 2/7/22.
//

#ifndef PROTOBOT_MACROS_H
#define PROTOBOT_MACROS_H

#define SERIAL_PORT  "/dev/ttyS0"   // serial port location, on RPI use /dev/serial0
#define BAUD_RATE    B115200        // roboclaw encoder baud rate
#define TIMEOUT_DS   1              // timeout period, 1 ds = 100 ms (not used at the moment)
#define TIMEOUT_MS   12             // Actual encoder timeout, according to this library: https://github.com/bmegli/roboclaw
#define RETRIES      3              // number of retries
#define M1FORWARD    0              // M1 Forward command
#define M2FORWARD    4              // M2 Forward command
#define M1DRIVE      35             // drive M1 with signed speed
#define M2DRIVE      36             // drive M2 with signed speed
#define M1_READ_ENCODER_SPEED 18    // read M1 encoder speed
#define M2_READ_ENCODER_SPEED 19    // read M2 encoder speed
#define ROBOCLAW_ACK_BYTES 1        // number of acknowledgement bytes
#define ROBOCLAW_ACK_BYTE  0xFF     // roboclaw acknowledgement byte upon successful command
#define DEBUG                       // debug mode

#endif //PROTOBOT_MACROS_H
