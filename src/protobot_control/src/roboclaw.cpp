//
// Created by jacob on 2/7/22.
//

#include<iostream>
#include<cstdlib>
#include<cstring>
#include<fcntl.h>
#include<errno.h>
#include<termios.h>
#include<unistd.h> // serial read() and write() defined here
#include<sys/time.h>
#include<assert.h>
#include<math.h>
#include "roboclaw.h"
#include "macros.h"

roboclaw::roboclaw() {

    // implement subroutine to verify serial port permissions

}

/* Send and execute commands to encoders. Returns -1 or RETRIES on failure, 1 on success. Commands will be sent up to max RETRIES.
   Successful writes to encoders are then polled to check if data is available to be read back. Once data is available, the data
   is read back, then the user's I/O buffers are cleared. */

int roboclaw::SendCommands(uint8_t* data, int writeBytes, int readBytes) {

    int r, writeFlag, readFlag, flushFlag, waitStatus;

    for(r=0; r<RETRIES; ++r) {

        for( ; r<RETRIES; ++r) {

            writeFlag = WriteToEncoders(data, writeBytes);
            if(writeFlag == -1) return -1;

            waitStatus = WaitReadStatus(readBytes, TIMEOUT_MS);

            if(waitStatus == 1) break; // data is available to be read back
            if(waitStatus == -1 || waitStatus == 0) return -1;

            flushFlag = ClearIOBuffers();
            if(flushFlag == -1) return -1;
        }

        if(r >= RETRIES) return -1;

        readFlag = ReadFromEncoders(readBytes);
        if(readFlag > 0) return 1;
        if(readFlag == -1) return -1;

        assert(readFlag == readBytes);

        flushFlag = ClearIOBuffers();
        if(flushFlag == -1) return -1;

    }

    return r; // max retries exceeded
}

int roboclaw::ClearIOBuffers() {

    return tcflush(serialPort, TCIOFLUSH);
}

/* Setup Roboclaw motor encoders. Open serial port for reading and writing, configure for 1 stop bit, 8 bits per byte, no parity checking
   as per the roboclaw user manual. Send and receive raw bytes only. Set baud rate of sending and receiving end. */

void roboclaw::SetupEncoders() {

    serialPort = open(SERIAL_PORT, O_RDWR | O_NOCTTY); // enable read & write, disable controlling terminal

    fcntl(serialPort, F_SETFL, 0); // set to blocking mode (for reads)
    // fcntl(serialPort, F_SETFL, FNDELAY); // non-blocking mode (for reads)

    if (serialPort < 0) {

        printf("Error %i from open: %s\n", errno, strerror(errno));
        exit(1);
    }

    if (tcgetattr(serialPort, &tty) != 0) {

        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        exit(1);
    }

    // set necessary bits

    tty.c_cflag &= ~PARENB; 	    // disable parity
    tty.c_cflag &= ~CSTOPB; 	    // set 1 stop bit
    tty.c_cflag |= CS8;			    // set 8 bits per byte
    tty.c_cflag &= ~CRTSCTS;	    // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;	// enable read from device, ignore ctrl lines

    tty.c_lflag &= ~ICANON;		    // disable canonical mode

    // next 4 flags are probably already disabled from above flag

    tty.c_lflag &= ~ECHO;		    // disable echo of commands
    tty.c_lflag &= ~ECHOE;		    // disable erasure
    tty.c_lflag &= ~ECHONL;		    // disable new-line echo
    tty.c_lflag &= ~ISIG;		    // disable special character handling

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 0;            // set min timeout period
    tty.c_cc[VMIN] = 1;             // set min number of characters to be read in

    // set baud rates

    cfsetispeed(&tty, BAUD_RATE);
    cfsetospeed(&tty, BAUD_RATE);

    // save flag settings

    if(tcsetattr(serialPort, TCSANOW, &tty) != 0)
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
}

/* Close serial port file descriptor. */

void roboclaw::CloseEncoders() {

    close(serialPort);
}

/* Write commands to encoders, check flag status. Returns number of bytes successfully sent to encoders (writeFlag > 0).
   Returns -1 on failure. */

int roboclaw::WriteToEncoders(uint8_t* data, int nBytes) {

    int writeFlag = write(serialPort, data, nBytes);

    if (writeFlag != nBytes) return -1;

    return writeFlag;
}

/* Check to see if roboclaw encoders sent back data (ACK byte, encoder data, etc.). Poll serial port to check that
   data is available to be read back, within the specified timeout period. Returns 1 on success. Otherwise,
   return 0 or -1 signifying an error. */

int roboclaw::WaitReadStatus(int nBytes, int timeout_ms) {

    struct timeval tv;              // from sys/time.h, used to set timeout for roboclaw reads
    fd_set input;                   // contains all file descriptors used
    int ret;                        // select flag

    FD_ZERO(&input);                // initialize file descriptors
    FD_SET(serialPort, &input);     // set file descriptors

    tv.tv_sec = 0;
    tv.tv_usec = timeout_ms*1000;   // set timeout period

    if(tty.c_cc[VMIN] != nBytes) {

        tty.c_cc[VMIN] = nBytes;    // set minimum number of bytes to be read in

        if(tcsetattr(serialPort, TCSANOW, &tty) < 0) return -1; // save settings
    }

    ret = select(serialPort + 1, &input, NULL, NULL, &tv);

    if (FD_ISSET(serialPort, &input))
        return 1;

    return ret; // error, will return either 0 or -1. check errno
}

/* Reads available data from roboclaw buffer. returns the number of bytes specified by the user upon success (readFlag > 0).
   Returns -1 upon failure. */

int roboclaw::ReadFromEncoders(int nBytes) {

    for(int i=0; i<MAX_BUF; i++)     // initialize buffer
        buf[i] = 0x00;

    int readFlag = read(serialPort, &buf, nBytes);

    if (readFlag != nBytes) return -1;

    return readFlag;
}

/* Get checksum calculation. the Checksum is calculated using Cyclic Redundancy Check (CRC), which uses the encoder address,
   commands and data sent to the encoders. Valid commands will be executed by the encoder, invalid commands will be discarded. */

uint32_t roboclaw::ValidateChecksum(uint8_t* packet, int nBytes) {

    int crc = 0;

    for(int byte=0; byte<nBytes; byte++) {

        crc = crc ^ ((uint32_t)packet[byte] << 8);

        for(uint8_t bit=0; bit<8; bit++) {

            if(crc & 0x8000) {

                crc = (crc << 1) ^ 0x1021;

            } else {

                crc = crc << 1;
            }
        }
    }

    return crc;
}

/* Move M1 motors. specify the motor address and desired speed value (0-127), then get checksum. Set commands
   and checksum in an array, send commands to be executed. */

void roboclaw::ForwardM1(uint8_t address, uint8_t value) {

    uint8_t get_crc[3] = {address, M1FORWARD, value};
    uint8_t data[5];

    uint16_t crc = ValidateChecksum(get_crc, 3);

    for(int i=0; i<3; i++)
        data[i] = get_crc[i];

    data[3] = crc >> 8;
    data[4] = crc;

    int cmdFlag = SendCommands(data, 5, 1);
}

/* Move M1 motors backwards */

void roboclaw::BackwardM1(uint8_t address, uint8_t value) {

    uint8_t get_crc[3] = {address, M1BACKWARD, value};
    uint8_t data[5];

    uint16_t crc = ValidateChecksum(get_crc, 3);

    for(int i=0; i<3; i++)
        data[i] = get_crc[i];

    data[3] = crc >> 8;
    data[4] = crc;

    int cmdFlag = SendCommands(data, 5, 1);
}

/* Move M2 motors. specify the motor address and desired speed value (0-127), then get checksum. Set commands
   and checksum in an array, send commands to be executed. */

void roboclaw::ForwardM2(uint8_t address, uint8_t value) {

    uint8_t get_crc[3] = {address, M2FORWARD, value};
    uint8_t data[5];

    uint16_t crc = ValidateChecksum(get_crc, 3);

    for(int i=0; i<3; i++)
        data[i] = get_crc[i];

    data[3] = crc >> 8;
    data[4] = crc;

    int cmdFlag = SendCommands(data, 5, 1);
}

/* Move M2 motors backwards */

void roboclaw::BackwardM2(uint8_t address, uint8_t value) {

    uint8_t get_crc[3] = {address, M2BACKWARD, value};
    uint8_t data[5];

    uint16_t crc = ValidateChecksum(get_crc, 3);

    for(int i=0; i<3; i++)
        data[i] = get_crc[i];

    data[3] = crc >> 8;
    data[4] = crc;

    int cmdFlag = SendCommands(data, 5, 1);
}

void roboclaw::ReadEncoderSpeedM1(uint8_t address) {

    uint8_t get_crc[2] = {address, M1_READ_ENCODER_SPEED};
    uint8_t data[7];

    for(int i=0; i<2; i++)
        data[i] = get_crc[i];

    int cmdFlag = SendCommands(data, 2, 7);
}

void roboclaw::ReadEncoderSpeedM2(uint8_t address) {

    uint8_t get_crc[2] = {address, M2_READ_ENCODER_SPEED};
    uint8_t data[7];

    for(int i=0; i<2; i++)
        data[i] = get_crc[i];

    int cmdFlag = SendCommands(data, 2, 7);
}

void roboclaw::SendCommandToWheels(double* cmd) {

    uint8_t cmd_send[6] = {0};

    // convert cmd_vel to a usable command between 0-127

    for(int i=0; i<=5; i++)
        cmd_send[i] = ScaleCommand(cmd[i]);

    // if positive, move motors forward. if negative, move backwards

    if(cmd[0] > 0)
        ForwardM1(0x80, cmd_send[0]);
    else
        BackwardM1(0x80, cmd_send[0]);

    if(cmd[1] > 0)
        ForwardM1(0x81, cmd_send[1]);
    else
        BackwardM1(0x81, cmd_send[1]);

    if(cmd[2] > 0)
        ForwardM1(0x82, cmd_send[2]);
    else
        BackwardM1(0x82, cmd_send[2]);

    if(cmd[3] > 0)
        ForwardM2(0x80, cmd_send[3]);
    else
        BackwardM2(0x80, cmd_send[3]);

    if(cmd[4] > 0)
        ForwardM2(0x81, cmd_send[4]);
    else
        BackwardM2(0x81, cmd_send[4]);

    if(cmd[5] > 0)
        ForwardM2(0x82, cmd_send[5]);
    else
        BackwardM2(0x82, cmd_send[5]);
}

void roboclaw::GetVelocityFromWheels(double* vel) {

    // return positive or negative value from encoders, depending on direction

    ReadEncoderSpeedM1(0x80);
    vel[0] = ConvertPulsesToRadians((double) (buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]));
    if(buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM1(0x81);
    vel[1] = ConvertPulsesToRadians((double) (buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]));
    if(buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM1(0x82);
    vel[2] = ConvertPulsesToRadians((double) (buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]));
    if(buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM2(0x80);
    vel[3] = ConvertPulsesToRadians((double) (buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]));
    if(buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM2(0x81);
    vel[4] = ConvertPulsesToRadians((double) (buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]));
    if(buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM2(0x82);
    vel[5] = ConvertPulsesToRadians((double) (buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]));
    if(buf[4] == 1) vel[0] = -vel[0];
}

// only for positive commands right now, 0 - 2m/s

uint8_t roboclaw::ScaleCommand(double cmd) {

    double res = (fabs(cmd)/16.667)*MAX_M1M2_VALUE;

    if(res >= MAX_M1M2_VALUE)
        res = MAX_M1M2_VALUE;

    return res; // setup for teleop_twist for now, max teleop_twist is 16.667
}

double roboclaw::ConvertPulsesToRadians(double vel) {

    return (vel/119.3697); // convert from QPPS to rad/s, (750 p/rev)(1/(2*pi))
}