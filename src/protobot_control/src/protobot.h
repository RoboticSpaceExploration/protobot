//
// Created by jacob on 2/7/22.
//

#ifndef PROTOBOT_PROTOBOT_H
#define PROTOBOT_PROTOBOT_H

#endif //PROTOBOT_PROTOBOT_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

class protobot : public hardware_interface::RobotHW {

public:
    protobot();
    virtual void Read() = 0;    // changed to capital, unistd.h has read()
    virtual void Write() = 0;   // changed to capital, unistd.h has write()
    ros::Time get_time();
    ros::Duration get_period();
private:
    static constexpr double BILLION = 1000000000.0;
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    void registerStateHandlers();
    void registerJointVelocityHandlers();
    ros::Duration elapsed_time;
    struct timespec last_time;
    struct timespec current_time;
protected:
    double cmd[6];
    double pos[6];
    double vel[6];
    double eff[6];
};