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

#include "../include/protobot.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "handle");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);
    spinner.start();

    ROS_INFO_STREAM("Loading protobot_control_hw_node");
    
    settings es_main;

    pb::protobot robot(&es_main);
    controller_manager::ControllerManager cm(&robot);
    roboclaw rb(&es_main);
    ros::Rate rate(es_main.loop_frequency);

    ROS_INFO("Setting Yaml parameters for serial port");
    robot.setYamlParameters(&nh);

    ROS_INFO("Initializing roboclaw motor encoders");
    rb.SetupEncoders();

    while (ros::ok()) {
        robot.readTopicWriteToEncoders(&rb);
        cm.update(robot.get_time(), robot.get_period());
        robot.readFromEncoders(&rb);
        rate.sleep();
    }

    ROS_INFO("Shutting down roboclaw motor encoders");
    rb.CloseEncoders();

    return 0;
}

pb::protobot::protobot(settings* es_ptr) {
    ROS_INFO("Registering ros_control handlers");
    es = es_ptr;
    registerStateHandlers();
    registerJointVelocityHandlers();

    clock_gettime(CLOCK_MONOTONIC, &last_time);

    for (int i = 0; i <= 5; i++)
        cmd[i] = vel[i] = pos[i] = eff[i] = 0;
}

pb::protobot::~protobot() {
}

void pb::protobot::registerStateHandlers() {
    hardware_interface::JointStateHandle state_handle_a(
        es->rightJoints[0], &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b(
        es->rightJoints[1], &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    hardware_interface::JointStateHandle state_handle_c(
        es->rightJoints[2], &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_c);

    hardware_interface::JointStateHandle state_handle_d(
        es->leftJoints[0], &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_d);

    hardware_interface::JointStateHandle state_handle_e(
        es->leftJoints[1], &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_e);

    hardware_interface::JointStateHandle state_handle_f(
        es->leftJoints[2], &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_f);

    registerInterface(&jnt_state_interface);
}

void pb::protobot::registerJointVelocityHandlers() {
    hardware_interface::JointHandle vel_handle_a(
        jnt_state_interface.getHandle(es->rightJoints[0]), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_a);

    hardware_interface::JointHandle vel_handle_b(
        jnt_state_interface.getHandle(es->rightJoints[1]), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_b);

    hardware_interface::JointHandle vel_handle_c(
        jnt_state_interface.getHandle(es->rightJoints[2]), &cmd[2]);
    jnt_vel_interface.registerHandle(vel_handle_c);

    hardware_interface::JointHandle vel_handle_d(
        jnt_state_interface.getHandle(es->leftJoints[0]), &cmd[3]);
    jnt_vel_interface.registerHandle(vel_handle_d);

    hardware_interface::JointHandle vel_handle_e(
        jnt_state_interface.getHandle(es->leftJoints[1]), &cmd[4]);
    jnt_vel_interface.registerHandle(vel_handle_e);

    hardware_interface::JointHandle vel_handle_f(
        jnt_state_interface.getHandle(es->leftJoints[2]), &cmd[5]);
    jnt_vel_interface.registerHandle(vel_handle_f);

    registerInterface(&jnt_vel_interface);
}

void pb::protobot::readTopicWriteToEncoders(roboclaw* rb) {
    ROS_INFO_STREAM("READING JOINT STATES FROM ROS");
    printDebugInfo("SENDING CMD_VEL TO", cmd);

    rb->SendCommandToWheels(cmd);
}


void pb::protobot::readFromEncoders(roboclaw* rb) {
    rb->GetVelocityFromWheels(vel);

    ROS_INFO_STREAM("READING JOINT STATES FROM MOTOR ENCODERS");
    printDebugInfo("VEL FROM", vel);
}

ros::Time pb::protobot::get_time() {
    return ros::Time::now();
}

ros::Duration pb::protobot::get_period() {
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time =
            ros::Duration(current_time.tv_sec - last_time.tv_sec
            + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
    last_time = current_time;

    return elapsed_time;
}

void pb::protobot::printDebugInfo(std::string name, double* data) {
    ROS_INFO_STREAM(name << " RIGHT_FRONT_WHEEL_JOINT "  << data[0]);
    ROS_INFO_STREAM(name << " RIGHT_MIDDLE_WHEEL_JOINT " << data[1]);
    ROS_INFO_STREAM(name << " RIGHT_BACK_WHEEL_JOINT "   << data[2]);
    ROS_INFO_STREAM(name << " LEFT_FRONT_WHEEL_JOINT "   << data[3]);
    ROS_INFO_STREAM(name << " LEFT_MIDDLE_WHEEL_JOINT "  << data[4]);
    ROS_INFO_STREAM(name << " LEFT_BACK_WHEEL_JOINT "    << data[5]);
}

void pb::protobot::setYamlParameters(ros::NodeHandle* nh) {
    int exitFlag = false;
    for (int i = 0; i < 2; i++) {
        es->rightAddr[i] = 0;
        es->leftAddr[i] = 0;
    }

    nh->getParam("/wheel_encoders/serial_port", es->serialPortAddr);
    nh->getParam("/wheel_encoders/send_command_retries", es->retries);
    nh->getParam("/wheel_encoders/encoder_timeout_ms", es->timeout_ms);
    nh->getParam("/wheel_encoders/loop_frequency", es->loop_frequency);
    nh->getParam("/wheel_encoders/baud_rate", es->baud_rate);
    nh->getParam("/wheel_encoders/right_wheel", rightJointList);
    nh->getParam("/wheel_encoders/left_wheel", leftJointList);
    nh->getParam("/wheel_encoders/right_addr", rightJointAddrList);
    nh->getParam("/wheel_encoders/left_addr", leftJointAddrList);

    for (int i = 0; i <= 2; i++) {
        es->rightJoints[i] = static_cast<std::string> (rightJointList[i]);
        es->leftJoints[i] = static_cast<std::string> (leftJointList[i]);
        es->rightAddr[i] = static_cast<int> (rightJointAddrList[i]);
        es->leftAddr[i] = static_cast<int> (leftJointAddrList[i]);

        if (es->rightJoints[i] == "") {
            ROS_ERROR("Right joint [%d] : Incorrect number of "
                      "joints specified in YAML file", i);
            exitFlag = true;
        }

        if (es->rightAddr[i] < 128 || es->rightAddr[i] > 135) {
            ROS_ERROR("Right address [%d] : Incorrect address "
                      "specified in YAML file", i);
            exitFlag = true;
        }

        if (es->leftJoints[i] == "") {
            ROS_ERROR("Left Joint [%d] : Incorrect number of "
                      "joints specified in YAML file", i);
            exitFlag = true;
        }

        if (es->leftAddr[i] < 128 || es->leftAddr[i] > 135) {
            ROS_ERROR("Left address [%d] : Incorrect address "
                      "specified in YAML file", i);
            exitFlag = true;
        }
    }

    if (exitFlag)
        exit(EXIT_FAILURE);
}
