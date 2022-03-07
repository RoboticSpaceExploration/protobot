#!/usr/bin/env python3.8
# ROS Node to convert a GPS waypoint published on the topic "waypoint" into a 2D Navigation Goal in SLAM to achieve autonomous navigation to a GPS Waypoint
# Converts Decimal GPS Coordinates of waypoint to ROS Position Vector relative to the current gps position of the robot
# Accounts for curvature of the earth using haversine formula

# Depends rospy, std_msgs, geographic_msgs, sensor_msgs, numpy
# Written by Alex McClung, 2015, alex.mcclung@hotmail.com, To be Released Open Source under Creative Commons Attribution Share-Alike Licence

import roslib
import rospy
from math import radians, cos, sin, asin, sqrt, pow, pi, atan2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geographic_msgs.msg import WayPoint

DEBUG = False
LATITUDE_CURRENT = 0.0
LONGITUDE_CURRENT = 0.0
LATITUDE_WAYPOINT = 0.0
LONGITUDE_WAYPOINT = 0.0
ALT_WAYPOINT = 0.0

EARTH_RADIUS = 6371000.0  # Metres
CURRENT_POSITION_X = 0.0
CURRENT_POSITION_Y = 0.0
CURRENT_POSITION_Z = 0.0

# True if there has been an update in the waypoint position
WAYPOINT_UPDATE_STATE = False

LAST_VALID_FIX_TIME = 0.0

GPS_VALIDITY_TIMEOUT = 10.0  # Seconds


# Returns distance to waypoint in Metres
def haversine_distance(latitude_current, longitude_current, latitude_waypoint, longitude_waypoint):
    # Convert into Radians to perform math
    latitude_waypoint, longitude_waypoint, latitude_current, longitude_current = map(
        radians, [latitude_waypoint, longitude_waypoint, latitude_current, longitude_current])
    a = pow(sin((latitude_waypoint - latitude_current)/2), 2) + cos(latitude_current) * \
        cos(latitude_waypoint) * \
        pow(sin((longitude_waypoint - longitude_current)/2), 2)
    # Return calculated distance to waypoint in Metres
    return EARTH_RADIUS * 2.0 * asin(sqrt(a))


# Bearing to waypoint (degrees)
def bearing(latitude_current, longitude_current, latitude_waypoint, longitude_waypoint):
    # Convert into Radians to perform math
    latitude_waypoint, longitude_waypoint, latitude_current, longitude_current = map(
        radians, [latitude_waypoint, longitude_waypoint, latitude_current, longitude_current])
    distance_longitude = longitude_waypoint - longitude_current
    return atan2(sin(distance_longitude) * cos(latitude_waypoint), cos(latitude_current) * sin(latitude_waypoint) - (sin(latitude_current) * cos(latitude_waypoint) * cos(distance_longitude)))


# GPS Coordinate recieved from ROS topic, run this function
def gps_subscriber(gps_message):
    # If there is a GPS fix (Either Augmented or Unaugmented)
    if gps_message.status.status > -1:
        global LATITUDE_CURRENT
        global LONGITUDE_CURRENT
        global LAST_VALID_FIX_TIME

        LAST_VALID_FIX_TIME = rospy.get_time()
        LATITUDE_CURRENT = gps_message.latitude
        LONGITUDE_CURRENT = gps_message.longitude
        if DEBUG == True:
            rospy.loginfo(
                "GPS Fix Available, Latitude: %f, Longitude: %f", LATITUDE_CURRENT, LONGITUDE_CURRENT)

# Check to see if there has been a GPS fix within the last <gpsValidityTimeout> seconds


def gps_fix_is_valid():
    global GPS_VALIDITY_TIMEOUT

    if (rospy.get_time() - LAST_VALID_FIX_TIME) < GPS_VALIDITY_TIMEOUT:
        return True
    else:
        rospy.loginfo("GPS Fix Invalid! Last valid update was: %f seconds ago",
                      rospy.get_time() - LAST_VALID_FIX_TIME)
        return False


# Odometry update recieved from ROS topic, run this function
def robot_pose_subscriber(pose_message):
    global CURRENT_POSITION_X
    global CURRENT_POSITION_Y
    global CURRENT_POSITION_Z

    CURRENT_POSITION_X = pose_message.pose.pose.position.x
    CURRENT_POSITION_Y = pose_message.pose.pose.position.y
    CURRENT_POSITION_Z = pose_message.pose.pose.position.z


# Waypoint Command recieved from ROS topic, run this function
def waypoint_subscriber(waypoint_message):
    global WAYPOINT_UPDATE_STATE
    global LATITUDE_WAYPOINT
    global LONGITUDE_WAYPOINT
    global ALT_WAYPOINT

    WAYPOINT_UPDATE_STATE = True
    LATITUDE_WAYPOINT = waypoint_message.position.latitude
    LONGITUDE_WAYPOINT = waypoint_message.position.longitude
    ALT_WAYPOINT = waypoint_message.position.altitude

    rospy.loginfo(
        "Recieved Waypoint Command, Latitude: %f, Longitude: %f", LATITUDE_WAYPOINT, LONGITUDE_WAYPOINT)

    if gps_fix_is_valid() == True:  # If there is a valid GPS fix, publish nav goal to ROS
        pose_publisher()

# Convert absolute waypoint to vector relative to robot, then publish navigation goal to ROS


def pose_publisher():
    desired_pose = PoseStamped()
    desired_pose.header.frame_id = "/gps_link"
    desired_pose.header.stamp = rospy.Time.now()

    global CURRENT_POSITION_X
    global CURRENT_POSITION_Y
    global CURRENT_POSITION_Z
    global DEBUG

    if DEBUG:
        rospy.loginfo("LatWP: %f, LonWP: %f, LatCur: %f, LonCur: %f",
                      LATITUDE_WAYPOINT, LONGITUDE_WAYPOINT, LATITUDE_CURRENT, LONGITUDE_CURRENT)
    distance_to_waypoint = haversine_distance(
        LATITUDE_CURRENT, LONGITUDE_CURRENT, LATITUDE_WAYPOINT, LONGITUDE_WAYPOINT)
    bearing_to_wp = bearing(
        LATITUDE_CURRENT, LONGITUDE_CURRENT, LATITUDE_WAYPOINT, LONGITUDE_WAYPOINT)

    # Convert distance and angle to waypoint from Polar to Cartesian co-ordinates then add current position of robot odometry
    desired_pose.pose.position.x = CURRENT_POSITION_X + \
        (distance_to_waypoint * cos(bearing_to_wp))
    desired_pose.pose.position.y = CURRENT_POSITION_Y + \
        (distance_to_waypoint * sin(bearing_to_wp))
    # Assuming CurrPosZ is abslolute (eg barometer or GPS)
    desired_pose.pose.position.z = ALT_WAYPOINT - CURRENT_POSITION_Z
    desired_pose.pose.orientation.x = 0
    desired_pose.pose.orientation.y = 0
    desired_pose.pose.orientation.z = 0
    desired_pose.pose.orientation.w = 1

    # Publish Nav Goal to ROS topic
    navigation_goal_pub = rospy.Publisher(
        'move_base_simple/goal', PoseStamped, queue_size=10)
    navigation_goal_pub.publish(desired_pose)

    rospy.loginfo("GPS Fix is Valid! Setting Navigation Goal to: %f, %f, %f",
                  desired_pose.pose.position.x, desired_pose.pose.position.y, desired_pose.pose.position.z)
    rospy.loginfo("Robot is heading %f metres at a bearing of %f degrees",
                  distance_to_waypoint, (bearing_to_wp * 180/pi + 360) % 360)


def main():
    rospy.init_node('gps_2d_navigation_goal', anonymous=True)
    rospy.loginfo("Initiating GPS 2D Nav Goal Node.")

    while not rospy.is_shutdown():  # While ros comms are running smoothly
        # Subscribe to "pose", "fix" and "waypoint" ROS topics
        rospy.Subscriber("waypoint", WayPoint, waypoint_subscriber)
        rospy.Subscriber("gps/fix", NavSatFix, gps_subscriber)
        rospy.Subscriber("localization/filtered/pose_global",
                         PoseWithCovarianceStamped, robot_pose_subscriber)
        rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        tax
        pass
