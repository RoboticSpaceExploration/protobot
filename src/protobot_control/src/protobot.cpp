//
// Created by jacob on 2/7/22.
//

#include "protobot.h"
#include "roboclaw.h"

pb::protobot::protobot(roboclaw *rb)
{
    // initialize cmd & vel to zero

    for(int i=0; i<5; i++) {
        vel[i] = 0;
        cmd[i] = 0;
    }

    ROS_INFO("Registering ros_control handlers");
    registerStateHandlers();
    registerJointVelocityHandlers();

    ROS_INFO("Initializing roboclaw motor encoders");
    rb->SetupEncoders();

    clock_gettime(CLOCK_MONOTONIC, &last_time);
}

void pb::protobot::registerStateHandlers() {
    hardware_interface::JointStateHandle state_handle_a("right_front_wheel_pivot", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("right_mid_wheel_pivot", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    hardware_interface::JointStateHandle state_handle_c("right_back_wheel_pivot", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_c);

    hardware_interface::JointStateHandle state_handle_d("left_front_wheel_pivot", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_d);

    hardware_interface::JointStateHandle state_handle_e("left_mid_wheel_pivot", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_e);

    hardware_interface::JointStateHandle state_handle_f("left_back_wheel_pivot", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_f);

    registerInterface(&jnt_state_interface);
}

void pb::protobot::registerJointVelocityHandlers() {
    hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("right_front_wheel_pivot"), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_a);

    hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("right_mid_wheel_pivot"), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_b);

    hardware_interface::JointHandle vel_handle_c(jnt_state_interface.getHandle("right_back_wheel_pivot"), &cmd[2]);
    jnt_vel_interface.registerHandle(vel_handle_c);

    hardware_interface::JointHandle vel_handle_d(jnt_state_interface.getHandle("left_front_wheel_pivot"), &cmd[3]);
    jnt_vel_interface.registerHandle(vel_handle_d);

    hardware_interface::JointHandle vel_handle_e(jnt_state_interface.getHandle("left_mid_wheel_pivot"), &cmd[4]);
    jnt_vel_interface.registerHandle(vel_handle_e);

    hardware_interface::JointHandle vel_handle_f(jnt_state_interface.getHandle("left_back_wheel_pivot"), &cmd[5]);
    jnt_vel_interface.registerHandle(vel_handle_f);

    registerInterface(&jnt_vel_interface);
}


void pb::protobot::readCmdVelFromTopic()
{
      if(fabs(cmd[0]) > 0.005f || fabs(cmd[1]) > 0.005f)
      {
        ROS_INFO_STREAM("READING JOINT STATES FROM ROS");
        ROS_INFO("CMD_VEL RIGHT_FRONT_WHEEL_JOINT %f", cmd[0]);
        ROS_INFO("CMD_VEL RIGHT_MIDDLE_WHEEL_JOINT %f", cmd[1]);
        ROS_INFO("CMD_VEL RIGHT_BACK_WHEEL_JOINT %f", cmd[2]);
        ROS_INFO("CMD_VEL LEFT_FRONT_WHEEL_JOINT %f", cmd[3]);
        ROS_INFO("CMD_VEL RIGHT_MIDDLE_WHEEL_JOINT %f", cmd[4]);
        ROS_INFO("CMD_VEL LEFT_BACK_WHEEL_JOINT %f", cmd[5]);
      }
}


void pb::protobot::writeAndReadFromEncoders(roboclaw *rb)
{

    ROS_INFO_STREAM("SENDING COMMANDS TO WHEELS");
    ROS_INFO("CMD_VEL TO RIGHT_FRONT_WHEEL_JOINT %f", cmd[0]);
    ROS_INFO("CMD_VEL TO RIGHT_MIDDLE_WHEEL_JOINT %f", cmd[1]);
    ROS_INFO("CMD_VEL TO RIGHT_BACK_WHEEL_JOINT %f", cmd[2]);
    ROS_INFO("CMD_VEL TO LEFT_FRONT_WHEEL_JOINT %f", cmd[3]);
    ROS_INFO("CMD_VEL TO RIGHT_MIDDLE_WHEEL_JOINT %f", cmd[4]);
    ROS_INFO("CMD_VEL TO LEFT_BACK_WHEEL_JOINT %f", cmd[5]);

    rb->SendCommandToWheels(cmd);

    rb->GetVelocityFromWheels(vel);

    ROS_INFO_STREAM("READING JOINT STATES FROM MOTOR ENCODERS");
    ROS_INFO("VEL FROM RIGHT_FRONT_WHEEL_JOINT %f", vel[0]);
    ROS_INFO("VEL FROM RIGHT_MIDDLE_WHEEL_JOINT %f", vel[1]);
    ROS_INFO("VEL FROM RIGHT_BACK_WHEEL_JOINT %f", vel[2]);
    ROS_INFO("VEL FROM LEFT_FRONT_WHEEL_JOINT %f", vel[3]);
    ROS_INFO("VEL FROM RIGHT_MIDDLE_WHEEL_JOINT %f", vel[4]);
    ROS_INFO("VEL FROM LEFT_BACK_WHEEL_JOINT %f", vel[5]);
}


ros::Time pb::protobot::get_time()
{
    return ros::Time::now();
}

ros::Duration pb::protobot::get_period()
{
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time =
            ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
    last_time = current_time;

    return elapsed_time;
}

int main(int argc, char** argv)
{
    double hz = 20;
    double period = 1 / hz;
    ros::init(argc,argv,"handle");
    //ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);
    spinner.start();

    ROS_INFO_STREAM("Loading protobot_control_hw_node");

    roboclaw rb;
    pb::protobot robot(&rb);
    controller_manager::ControllerManager cm(&robot);

    // Control loop here
    ros::Rate rate(hz);
    while (ros::ok())
    {
        robot.readCmdVelFromTopic();
        cm.update(robot.get_time(), robot.get_period());
        robot.writeAndReadFromEncoders(&rb);
        rate.sleep();
    }

    return 0;
}
