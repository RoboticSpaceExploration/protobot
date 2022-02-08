//
// Created by jacob on 2/7/22.
//

#include "protobot.h"

protobot::protobot()
{
    ROS_INFO("Registering ros_control handlers");
    registerStateHandlers();
    registerJointVelocityHandlers();

    clock_gettime(CLOCK_MONOTONIC, &last_time);
}

void protobot::registerStateHandlers() {
    hardware_interface::JointStateHandle state_handle_a("right_wheel_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("left_wheel_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&jnt_state_interface);
}

void protobot::registerJointVelocityHandlers() {
    hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("right_wheel_joint"), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_a);

    hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("left_wheel_joint"), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_b);

    registerInterface(&jnt_vel_interface);
}
/*
void protobot::read()
{
//      if(fabs(cmd[0]) > 0.005f || fabs(cmd[1]) > 0.005f)
//      {
//        ROS_INFO_STREAM("READING");
//        ROS_INFO("CMD TO RIGHT_WHEEL_JOINT %f", cmd[0]);
//        ROS_INFO("CMD TO LEFT_WHEEL_JOINT %f", cmd[1]);
//      }
    double right_vel = cmd[0];
    double left_vel = cmd[1];
}
*/

/*
void protobot::write()
{
    vel[0] = cmd[0];
    vel[1] = cmd[1];
}
*/

ros::Time protobot::get_time()
{
    return ros::Time::now();
}

ros::Duration protobot::get_period()
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
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);
    spinner.start();

    ROS_INFO_STREAM("Loading protobot_control_node");

    protobot robot;
    controller_manager::ControllerManager cm(&robot);


    // Control loop here
    ros::Rate rate(hz);
    while (ros::ok())
    {
        robot.read();
        cm.update(robot.get_time(), robot.get_period());
        robot.write();
        rate.sleep();
    }

}
