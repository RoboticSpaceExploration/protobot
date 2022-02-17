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

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class OdometryTransform{
 public:
    OdometryTransform() {
        pub = nh.advertise<geometry_msgs
            ::PoseWithCovarianceStamped>("pose", 50);
        sub = nh.subscribe("nav_msgs/Odometry",
                           1000,
                           &OdometryTransform::poseMessageRecieved, this);
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
