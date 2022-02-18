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

#ifndef SRC_PROTOBOT_CONTROL_INCLUDE_PROTOBOT_H_
#define SRC_PROTOBOT_CONTROL_INCLUDE_PROTOBOT_H_

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <string>
#include "../include/roboclaw.h"
#include "../include/settings.h"

namespace pb {
class protobot : public hardware_interface::RobotHW {
 public:
    protobot();

    void readTopicWriteToEncoders(roboclaw* rb);
    void readFromEncoders(roboclaw* rb);
    void setYamlParameters(settings* es);

    ros::Time get_time();
    ros::Duration get_period();

 private:
    static constexpr double BILLION = 1000000000.0;
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    ros::NodeHandle nh;

    void registerStateHandlers();
    void registerJointVelocityHandlers();
    void printDebugInfo(std::string name, double* data);
    void setYamlParameters();

    ros::Duration elapsed_time;
    struct timespec last_time;
    struct timespec current_time;
    double cmd[6];
    double pos[6];
    double vel[6];
    double eff[6];
};
}  // namespace pb

#endif  // SRC_PROTOBOT_CONTROL_INCLUDE_PROTOBOT_H_
