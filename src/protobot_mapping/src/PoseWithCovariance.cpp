#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <iomanip>

class PoseWithCovariance{
	public:
	PoseWithCovariance() {
		pub = nh.advertise<geometry_msgs::PoseWithCovariance>("pose", 50);
        	sub = nh.subscribe("nav_msgs/Odometry", 1000, &PoseWithCovariance::poseMessageRecieved, this);
}
	private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;


	void poseMessageRecieved(const nav_msgs::Odometry& msg) {
		geometry_msgs::PoseWithCovariance pose2;
		pose2 = msg.pose;
		pub.publish(pose2);
}
};
int main(int argc, char **argv) {
	ros::init(argc, argv, "node initialized");
	PoseWithCovariance pwc;
	ros::spin();
}
