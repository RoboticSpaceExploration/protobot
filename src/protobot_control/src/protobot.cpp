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
    ros::AsyncSpinner spinner(3);
    spinner.start();

    ROS_INFO_STREAM("Loading protobot_control_hw_node");

    pb::protobot robot;
    controller_manager::ControllerManager cm(&robot);

    settings* es_ptr = new settings;

    ROS_INFO("Setting Yaml parameters for serial port");
    robot.setYamlParameters(es_ptr);

    roboclaw rb(es_ptr);

    ROS_INFO("Initializing roboclaw motor encoders");
    rb.SetupEncoders();

    // Control loop here
    ros::Rate rate(es_ptr->loop_frequency);

    while (ros::ok()) {
        robot.readTopicWriteToEncoders(&rb);
        cm.update(robot.get_time(), robot.get_period());
        robot.readFromEncoders(&rb);
        rate.sleep();
    }

    delete es_ptr;

    ROS_INFO("Shutting down roboclaw motor encoders");
    rb.CloseEncoders();

    return 0;
}

pb::protobot::protobot() {
    ROS_INFO("Registering ros_control handlers");
    registerStateHandlers();
    registerJointVelocityHandlers();

    clock_gettime(CLOCK_MONOTONIC, &last_time);

    for (int i = 0; i < 5; i++) {
        cmd[i] = 0;
        vel[i] = 0;
    }
}

void pb::protobot::registerStateHandlers() {
    hardware_interface::JointStateHandle state_handle_a(
        "right_front_wheel_pivot", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b(
        "right_mid_wheel_pivot", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    hardware_interface::JointStateHandle state_handle_c(
        "right_back_wheel_pivot", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_c);

    hardware_interface::JointStateHandle state_handle_d(
        "left_front_wheel_pivot", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_d);

    hardware_interface::JointStateHandle state_handle_e(
        "left_mid_wheel_pivot", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_e);

    hardware_interface::JointStateHandle state_handle_f(
        "left_back_wheel_pivot", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_f);

    registerInterface(&jnt_state_interface);
}

void pb::protobot::registerJointVelocityHandlers() {
    hardware_interface::JointHandle vel_handle_a(
        jnt_state_interface.getHandle("right_front_wheel_pivot"), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_a);

    hardware_interface::JointHandle vel_handle_b(
        jnt_state_interface.getHandle("right_mid_wheel_pivot"), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_b);

    hardware_interface::JointHandle vel_handle_c(
        jnt_state_interface.getHandle("right_back_wheel_pivot"), &cmd[2]);
    jnt_vel_interface.registerHandle(vel_handle_c);

    hardware_interface::JointHandle vel_handle_d(
        jnt_state_interface.getHandle("left_front_wheel_pivot"), &cmd[3]);
    jnt_vel_interface.registerHandle(vel_handle_d);

    hardware_interface::JointHandle vel_handle_e(
        jnt_state_interface.getHandle("left_mid_wheel_pivot"), &cmd[4]);
    jnt_vel_interface.registerHandle(vel_handle_e);

    hardware_interface::JointHandle vel_handle_f(
        jnt_state_interface.getHandle("left_back_wheel_pivot"), &cmd[5]);
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
    ROS_INFO("%s RIGHT_FRONT_WHEEL_JOINT %f", name.c_str(), data[0]);
    ROS_INFO("%s RIGHT_MIDDLE_WHEEL_JOINT %f", name.c_str(), data[1]);
    ROS_INFO("%s RIGHT_BACK_WHEEL_JOINT %f", name.c_str(), data[2]);
    ROS_INFO("%s LEFT_FRONT_WHEEL_JOINT %f", name.c_str(), data[3]);
    ROS_INFO("%s RIGHT_MIDDLE_WHEEL_JOINT %f", name.c_str(), data[4]);
    ROS_INFO("%s LEFT_BACK_WHEEL_JOINT %f", name.c_str(), data[5]);
}

void pb::protobot::setYamlParameters(settings* es) {
    nh.getParam("/serial_port", es->serialPortAddr);
    nh.getParam("/send_command_retries", es->retries);
    nh.getParam("/encoder_timeout_ms", es->timeout_ms);
    nh.getParam("/max_read_buffer_size", es->max_buf_size);
    nh.getParam("/loop_frequency", es->loop_frequency);
}
