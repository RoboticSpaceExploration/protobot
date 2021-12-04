#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iomanip>

class OdometryTransform{
	public:
	OdometryTransform() {
		pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 50);
        	sub = nh.subscribe("nav_msgs/Odometry", 1000, &OdometryTransform::poseMessageRecieved, this);
}
	private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;


	void poseMessageRecieved(const nav_msgs::Odometry& msg) {
		geometry_msgs::PoseWithCovarianceStamped pose2;
		pose2.header = msg.header;
		pose2.pose = msg.pose;
		pub.publish(pose2);
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "node initialized");
	OdometryTransform pwc;
	ros::spin();
}
