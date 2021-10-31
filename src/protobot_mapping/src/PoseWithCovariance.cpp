#include <ros/ros.h>
#include <nav_messages/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <iomanip>

class PoseWithCovariance{
	public:
	PoseWithCovariance() {
		pub = nh.advertise<geometry_msgs::PoseWithCovariance>("pose", 50);
        	sub = nh.subscribe("nav_messages/Odometry", 1000, &PoseMessageRecieved);

}
	private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;


	void poseMessageRecieved(const nav_messages::Odometry& msg) {
		geometry_msgs::PoseWithCovariance pose;
		pose = msg.PoseWithCovariance;
		sub.publish(pose);
}
};
int main(int argc, char **argv) {
	ros::init(argc, argv, "node initialized");
	PoseWithCovariance pwc;
	ros::spin();
}
