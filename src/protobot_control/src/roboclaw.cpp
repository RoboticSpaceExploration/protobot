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
#include "../include/roboclaw.h"

#define RECOMBINE_BUFFER(X) \
(((X)[3]) << 24 | ((X)[2]) << 16 | ((X)[1]) << 8 | ((X)[0]))

roboclaw::roboclaw(settings* es_protobot) {
    zeroCmdVelCount = 0;
    es = es_protobot;
}

/* Send and execute commands to encoders. Returns -1 or es.retries on failure, 1 on success. Commands will be sent up to max es.retries.
   Successful writes to encoders are then polled to check if data is available to be read back. Once data is available, the data
   is read back, then the user's I/O buffers are cleared. */

int roboclaw::SendCommands(uint8_t* data, int writeBytes, int readBytes) {
    int r, writeFlag, readFlag, flushFlag, waitStatus;

    for (r=0; r < es->retries; ++r) {
        for ( ; r < es->retries; ++r) {
            writeFlag = WriteToEncoders(data, writeBytes);
            if (writeFlag == -1) return -1;

            waitStatus = WaitReadStatus(readBytes, es->timeout_ms);

            if (waitStatus == 1) break;  // data is available to be read back
            if (waitStatus == -1 || waitStatus == 0) return -1;

            flushFlag = ClearIOBuffers();
            if (flushFlag == -1) return -1;
        }

        if (r >= es->retries) return -1;

        readFlag = ReadFromEncoders(readBytes);
        if (readFlag > 0) return 1;
        if (readFlag == -1) return -1;

        assert(readFlag == readBytes);

        flushFlag = ClearIOBuffers();
        if (flushFlag == -1) return -1;
    }

    return r;  // max retries exceeded
}

int roboclaw::ClearIOBuffers() {
    return tcflush(serialPort, TCIOFLUSH);
}

/* Setup Roboclaw motor encoders. Open serial port for reading and writing, configure for 1 stop bit, 8 bits per byte, no parity checking
   as per the roboclaw user manual. Send and receive raw bytes only. Set baud rate of sending and receiving end. */

void roboclaw::SetupEncoders() {
    // enable read & write, disable controlling terminal
    serialPort = open(es->serialPortAddr.c_str(), O_RDWR | O_NOCTTY);

    fcntl(serialPort, F_SETFL, 0);  // set to blocking mode (for reads)

    if (serialPort < 0) {
        ROS_INFO("Could not open %s: ", es->serialPortAddr.c_str());
        ROS_INFO("Error %i from open: %s\n", errno, strerror(errno));
        exit(1);
    }

    if (tcgetattr(serialPort, &tty) != 0) {
        ROS_INFO("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        exit(1);
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
    tty.c_cc[VMIN]  = 1;

    // set baud rates

    cfsetispeed(&tty, BAUD_RATE);
    cfsetospeed(&tty, BAUD_RATE);

    // save flag settings

    if (tcsetattr(serialPort, TCSANOW, &tty) != 0)
        ROS_INFO("Error %i from tcsetattr: %s\n", errno, strerror(errno));

    ClearIOBuffers();
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
    struct timeval tv;
    fd_set input;
    int ret;

    FD_ZERO(&input);
    FD_SET(serialPort, &input);

    tv.tv_sec = 0;
    tv.tv_usec = timeout_ms*1000;

    if (tty.c_cc[VMIN] != nBytes) {
        tty.c_cc[VMIN] = nBytes;

        if (tcsetattr(serialPort, TCSANOW, &tty) < 0)
            return -1;  // save settings
    }

    ret = select(serialPort + 1, &input, NULL, NULL, &tv);

    if (FD_ISSET(serialPort, &input))
        return 1;

    return ret;
}

/* Reads available data from roboclaw buffer. returns the number of bytes specified by the user upon success (readFlag > 0).
   Returns -1 upon failure. */

int roboclaw::ReadFromEncoders(int nBytes) {
    for (int i = 0; i < es->max_buf_size; i++)
        buf[i] = 0x00;

    int readFlag = read(serialPort, &buf, nBytes);

    if (readFlag != nBytes) return -1;

    return readFlag;
}

/* Get checksum calculation. the Checksum is calculated using Cyclic Redundancy Check (CRC), which uses the encoder address,
   commands and data sent to the encoders. Valid commands will be executed by the encoder, invalid commands will be discarded. */

uint32_t roboclaw::ValidateChecksum(uint8_t* packet, int nBytes) {
    int crc = 0;

    for (int byte = 0; byte < nBytes; byte++) {
        crc = crc ^ ((uint32_t)packet[byte] << 8);

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }

    return crc;
}

/* Move M1 motors. specify the motor address and desired speed value (0-127), then get checksum. Set commands
   and checksum in an array, send commands to be executed. */

void roboclaw::ForwardM1(uint8_t address, uint8_t value) {
    uint8_t get_crc[3] = {address, es->m1_forward, value};
    uint8_t data[5];

    uint16_t crc = ValidateChecksum(get_crc, 3);

    for (int i = 0; i < 3; i++)
        data[i] = get_crc[i];

    data[3] = crc >> 8;
    data[4] = crc;

    SendCommands(data, 5, 1);
}

/* Move M1 motors backwards */

void roboclaw::BackwardM1(uint8_t address, uint8_t value) {
    uint8_t get_crc[3] = {address, es->m1_backward, value};
    uint8_t data[5];

    uint16_t crc = ValidateChecksum(get_crc, 3);

    for (int i = 0; i < 3; i++)
        data[i] = get_crc[i];

    data[3] = crc >> 8;
    data[4] = crc;

    SendCommands(data, 5, 1);
}

/* Move M2 motors. specify the motor address and desired speed value (0-127), then get checksum. Set commands
   and checksum in an array, send commands to be executed. */

void roboclaw::ForwardM2(uint8_t address, uint8_t value) {
    uint8_t get_crc[3] = {address, es->m2_forward, value};
    uint8_t data[5];

    uint16_t crc = ValidateChecksum(get_crc, 3);

    for (int i = 0; i < 3; i++)
        data[i] = get_crc[i];

    data[3] = crc >> 8;
    data[4] = crc;

    SendCommands(data, 5, 1);
}

/* Move M2 motors backwards */

void roboclaw::BackwardM2(uint8_t address, uint8_t value) {
    uint8_t get_crc[3] = {address, es->m2_backward, value};
    uint8_t data[5];

    uint16_t crc = ValidateChecksum(get_crc, 3);

    for (int i = 0; i < 3; i++)
        data[i] = get_crc[i];

    data[3] = crc >> 8;
    data[4] = crc;

    SendCommands(data, 5, 1);
}

void roboclaw::ReadEncoderSpeedM1(uint8_t address) {
    uint8_t get_crc[2] = {address, es->m1_read_encoder_speed};
    uint8_t data[7];

    for (int i = 0; i < 2; i++)
        data[i] = get_crc[i];

    SendCommands(data, 2, 7);
}

void roboclaw::ReadEncoderSpeedM2(uint8_t address) {
    uint8_t get_crc[2] = {address, es->m2_read_encoder_speed};
    uint8_t data[7];

    for (int i = 0; i < 2; i++)
        data[i] = get_crc[i];

    SendCommands(data, 2, 7);
}

void roboclaw::SendCommandToWheels(double* cmd) {
    uint8_t cmd_send[6] = {0};

    // convert cmd_vel to a usable command between 0-127

    for (int i = 0; i <= 5; i++)
        cmd_send[i] = ScaleCommand(cmd[i]);

    // prevent zero velocity spamming from ros_control

    if (zeroCmdVelCount <= es->retries) {
        // if positive, move motors forward. if negative, move backwards
        if (cmd[0] >= 0)  // right_front
            ForwardM1(0x80, cmd_send[0]);
        else
            BackwardM1(0x80, cmd_send[0]);

        if (cmd[1] >= 0)  // right_middle
            ForwardM2(0x80, cmd_send[1]);
        else
            BackwardM2(0x80, cmd_send[1]);

        if (cmd[2] >= 0)  // right_back
            ForwardM1(0x81, cmd_send[2]);
        else
            BackwardM1(0x81, cmd_send[2]);

        if (cmd[3] >= 0)  // left_front
            ForwardM2(0x82, cmd_send[3]);
        else
            BackwardM2(0x82, cmd_send[3]);

        if (cmd[4] >= 0)  // left_middle
            ForwardM1(0x82, cmd_send[4]);
        else
            BackwardM1(0x82, cmd_send[4]);

        if (cmd[5] >= 0)  // left_back
            ForwardM2(0x81, cmd_send[5]);
        else
            BackwardM2(0x81, cmd_send[5]);
    }

    // if any of the cmd_vel are zero, increment counter

    if (cmd[0] == 0 || cmd[1] == 0 || cmd[2] == 0 ||
        cmd[3] == 0 || cmd[4] == 0 || cmd[5] == 0) {
        zeroCmdVelCount++;
    } else {
        zeroCmdVelCount = 0;  // reset counter
        cmd[0] = cmd[1] = cmd[2] = cmd[3] = cmd[4] = cmd[5] = 0;
    }
}

void roboclaw::GetVelocityFromWheels(double* vel) {
    // return positive or negative value from encoders, depending on direction
    ReadEncoderSpeedM1(0x80);  // right_front
    vel[0] = ConvertPulsesToRadians(
        static_cast<double> (RECOMBINE_BUFFER(buf)));
    if (buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM2(0x80);  // right_middle
    vel[1] = ConvertPulsesToRadians(
        static_cast<double> (RECOMBINE_BUFFER(buf)));
    if (buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM1(0x81);  // right_back
    vel[2] = ConvertPulsesToRadians(
        static_cast<double> (RECOMBINE_BUFFER(buf)));
    if (buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM2(0x82);  // left_front
    vel[3] = ConvertPulsesToRadians(
        static_cast<double> (RECOMBINE_BUFFER(buf)));
    if (buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM1(0x82);  // left_middle
    vel[4] = ConvertPulsesToRadians(
        static_cast<double> (RECOMBINE_BUFFER(buf)));
    if (buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM2(0x81);  // left_back
    vel[5] = ConvertPulsesToRadians(
        static_cast<double> (RECOMBINE_BUFFER(buf)));
    if (buf[4] == 1) vel[0] = -vel[0];
}

/* Scale command between 0-127 to be sent to encoders */

uint8_t roboclaw::ScaleCommand(double cmd) {
    double res = (fabs(cmd)/16.667)*es->max_m1m2_value;

    if (res >= es->max_m1m2_value)
        res = es->max_m1m2_value;

    return res;  // setup for teleop_twist for now, max teleop_twist is 16.667
}

/* will need to fix conversion in later push */

double roboclaw::ConvertPulsesToRadians(double vel) {
    return (vel/119.3697);  // convert from QPPS to rad/s, (750 p/rev)(1/(2*pi))
}
