#ifndef PROTOBOT_SETTINGS_H_
#define PROTOBOT_SETTINGS_H_

#include<string>

#define BAUD_RATE B115200

struct settings {

    std::string serialPortAddr    = "/dev/ttyAMA0";
    int timeout_ms                = 12;
    int retries                   = 3;
    uint8_t m1_forward            = 0;
    uint8_t m2_forward            = 4;
    uint8_t m1_backward           = 1;
    uint8_t m2_backward           = 5;
    uint8_t m1_read_encoder_speed = 18;
    uint8_t m2_read_encoder_speed = 19;
    int max_buf_size              = 100;
    double max_m1m2_value         = 127;
    double loop_frequency         = 10;

};

#endif //PROTOBOT_SETTINGS_H_
