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

#ifndef SRC_PROTOBOT_CONTROL_INCLUDE_SETTINGS_H_
#define SRC_PROTOBOT_CONTROL_INCLUDE_SETTINGS_H_

#include <string>

#define BAUD_RATE B115200

struct settings {
    std::string serialPortAddr    = "/dev/ttyAMA0";
    int timeout_ms                = 12;
    int retries                   = 3;
    int max_buf_size              = 100;
    uint8_t m1_forward            = 0;
    uint8_t m2_forward            = 4;
    uint8_t m1_backward           = 1;
    uint8_t m2_backward           = 5;
    uint8_t m1_read_encoder_speed = 18;
    uint8_t m2_read_encoder_speed = 19;
    double max_m1m2_value         = 127;
    double loop_frequency         = 10;
};

#endif  // SRC_PROTOBOT_CONTROL_INCLUDE_SETTINGS_H_
