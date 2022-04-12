#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdlib.h>

#define SPIRAL_LEN 20
#define INCREMENT_VALUE 1.5

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool moveToWaypoint( move_base_msgs::MoveBaseGoal goal ){

	MoveBaseClient ac( "move_base", true );

    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    ac.sendGoal( goal );
    ac.waitForResult();

    if( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED )
        return true;
    else
        return false;
}

int main( int argc, char** argv ){

    ros::init( argc, argv, "simple_navigation_goals" );

    MoveBaseClient ac( "move_base", true );

    while( !ac.waitForServer( ros::Duration( 5.0 ) ) ){
        ROS_INFO( "Waiting for the move_base action server to come up" );
    }

    float spiralValuesList[SPIRAL_LEN][1];
    float currValueX = 0;
    float currValueY = 0;
    int incrementCounter = 0;
    int xMultiplier = 2;
    int yMultiplier = 3;

    for( int index = 0; index < SPIRAL_LEN; index++ ){
        incrementCounter++;
        if(incrementCounter%2 == 1){
            currValueX = xMultiplier * INCREMENT_VALUE;
            if(xMultiplier < 0){
                xMultiplier =  (abs(xMultiplier) + 2);
            } else {
                xMultiplier =  -(abs(xMultiplier) + 2);
            }
            spiralValuesList[index][0] = currValueX;
            spiralValuesList[index][1] = 0;
        }
        if(incrementCounter%2 == 0){
            currValueY = yMultiplier * INCREMENT_VALUE;
            if(yMultiplier < 0){
                yMultiplier =  (abs(yMultiplier) + 2);
            } else {
                yMultiplier =  -(abs(yMultiplier) + 2);
            }
            spiralValuesList[index][0] = 0;
            spiralValuesList[index][1] = currValueY;
        }
    }

    move_base_msgs::MoveBaseGoal goal[SPIRAL_LEN];

    for( int waypointIndex = 0; waypointIndex < SPIRAL_LEN; waypointIndex++ ){
        goal[waypointIndex].target_pose.pose.position.x = spiralValuesList[waypointIndex][0];
        goal[waypointIndex].target_pose.pose.position.y = spiralValuesList[waypointIndex][1];

        goal[waypointIndex].target_pose.pose.orientation.x = 0;
        goal[waypointIndex].target_pose.pose.orientation.y = 0;
        goal[waypointIndex].target_pose.pose.orientation.z = 0;
        goal[waypointIndex].target_pose.pose.orientation.w = 0;
    }

    ROS_INFO( "Sending goal" );

    for( int waypointIndex = 0 ; waypointIndex < SPIRAL_LEN ; waypointIndex++ ){
        if( moveToWaypoint( goal[waypointIndex] ) ){
            ROS_INFO( "Goal %d reached", waypointIndex );
        }
        else{
            ROS_INFO( "Goal %d not reached", waypointIndex );
        }
    }
    return 0;
}


