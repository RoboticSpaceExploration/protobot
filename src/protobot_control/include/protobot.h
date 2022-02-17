#ifndef SRC_PROTOBOT_CONTROL_INCLUDE_PROTOBOT_H_
#define SRC_PROTOBOT_CONTROL_INCLUDE_PROTOBOT_H_

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <string>
#include "roboclaw.h"
#include "settings.h"

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
}

#endif  //SRC_PROTOBOT_CONTROL_INCLUDE_PROTOBOT_H_
