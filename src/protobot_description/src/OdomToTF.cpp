#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void poseCallback(const nav_msgs::Odometry& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Transformer transformer;
  geometry_msgs::Pose pose = msg.pose.pose;
  
  transformer.transformPose("base_link", pose, pose);

  transform.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z) );
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom","base_link"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/camera/odom/sample", 10, &poseCallback);

  ros::spin();
  return 0;
};

