#!/usr/bin/env python3.8
#ROS Node to convert a GPS waypoint published on the topic "waypoint" into a 2D Navigation Goal in SLAM to achieve autonomous navigation to a GPS Waypoint
#Converts Decimal GPS Coordinates of waypoint to ROS Position Vector relative to the current gps position of the robot
#Accounts for curvature of the earth using haversine formula

#Depends rospy, std_msgs, geographic_msgs, sensor_msgs, numpy
#Written by Alex McClung, 2015, alex.mcclung@hotmail.com, To be Released Open Source under Creative Commons Attribution Share-Alike Licence

import roslib
import rospy
from math import radians, cos, sin, asin, sqrt, pow, pi, atan2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geographic_msgs.msg import WayPoint

debug = False
latCur = 0.0
lonCur = 0.0
latWP = 0.0
lonWP = 0.0
altWP = 0.0

earthRadius = 6371000.0 #Metres
currPosX = 0.0
currPosY = 0.0
currPosZ = 0.0

WPUpdateState = False #True if there has been an update in the waypoint position

lastValidFixTime = 0.0

gpsValidityTimeout = 10.0 #Seconds

def haversineDistance(latCur, lonCur, latWP, lonWP): #Returns distance to waypoint in Metres
	latWP, lonWP, latCur, lonCur = map(radians, [latWP, lonWP, latCur, lonCur]) #Convert into Radians to perform math
	a = pow(sin((latWP - latCur)/2),2) + cos(latCur) * cos(latWP) * pow(sin((lonWP - lonCur)/2),2)
	return earthRadius * 2.0 * asin(sqrt(a))  #Return calculated distance to waypoint in Metres
	
def bearing(latCur, lonCur, latWP, lonWP): #Bearing to waypoint (degrees)
	latWP, lonWP, latCur, lonCur = map(radians, [latWP, lonWP, latCur, lonCur]) #Convert into Radians to perform math
	dLon = lonWP - lonCur
	return atan2(sin(dLon) * cos(latWP), cos(latCur) * sin(latWP) - (sin(latCur) * cos(latWP) * cos(dLon)))

def gpsSubscriber(gpsMsg): #GPS Coordinate recieved from ROS topic, run this function
	if gpsMsg.status.status > -1: #If there is a GPS fix (Either Augmented or Unaugmented)
		global latCur
		global lonCur
		global lastValidFixTime
		
		lastValidFixTime = rospy.get_time()		
		latCur = gpsMsg.latitude
		lonCur = gpsMsg.longitude
		if debug == True:
			rospy.loginfo("GPS Fix Available, Latitude: %f, Longitude: %f", latCur, lonCur)

def gpsFixIsValid(): #Check to see if there has been a GPS fix within the last <gpsValidityTimeout> seconds
		global gpsValidityTimeout
		
		if (rospy.get_time()- lastValidFixTime) < gpsValidityTimeout:
			return True
		else:
			rospy.loginfo("GPS Fix Invalid! Last valid update was: %f seconds ago", rospy.get_time()- lastValidFixTime)
			return False

def robotPoseSubscriber(poseMsg): #Odometry update recieved from ROS topic, run this function
	global currPosX
	global currPosY
	global currPosZ

	currPosX = poseMsg.pose.pose.position.x
	currPosY = poseMsg.pose.pose.position.y
	currPosZ = poseMsg.pose.pose.position.z

def waypointSubscriber(WPMsg): #Waypoint Command recieved from ROS topic, run this function
	global waypointUpdateState
	global latWP
	global lonWP
	global altWP

	WPUpdateState = True
	latWP = WPMsg.position.latitude
	lonWP = WPMsg.position.longitude
	altWP = WPMsg.position.altitude
	
	rospy.loginfo("Recieved Waypoint Command, Latitude: %f, Longitude: %f", latWP, lonWP)
	
	if gpsFixIsValid() == True: #If there is a valid GPS fix, publish nav goal to ROS
		posePublisher()

def posePublisher(): #Convert absolute waypoint to vector relative to robot, then publish navigation goal to ROS
	desiredPose = PoseStamped()
	desiredPose.header.frame_id = "/gps_link"
	desiredPose.header.stamp = rospy.Time.now()

	global currPosX
	global currPosY
	global currPosZ
	global debug
	
	if debug:
		rospy.loginfo("LatWP: %f, LonWP: %f, LatCur: %f, LonCur: %f", latWP, lonWP, latCur, lonCur)
	distToWP = haversineDistance(latCur, lonCur, latWP, lonWP)
	bearingToWP = bearing(latCur, lonCur, latWP, lonWP)
	
	desiredPose.pose.position.x = currPosX + (distToWP * cos(bearingToWP)) #Convert distance and angle to waypoint from Polar to Cartesian co-ordinates then add current position of robot odometry 
	desiredPose.pose.position.y = currPosY + (distToWP * sin(bearingToWP))
	desiredPose.pose.position.z = altWP - currPosZ #Assuming CurrPosZ is abslolute (eg barometer or GPS)
	desiredPose.pose.orientation.x = 0
	desiredPose.pose.orientation.y = 0
	desiredPose.pose.orientation.z = 0
	desiredPose.pose.orientation.w = 1

	navGoalPub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10) #Publish Nav Goal to ROS topic
	navGoalPub.publish(desiredPose)
	
	rospy.loginfo("GPS Fix is Valid! Setting Navigation Goal to: %f, %f, %f", desiredPose.pose.position.x, desiredPose.pose.position.y, desiredPose.pose.position.z)
	rospy.loginfo("Robot is heading %f metres at a bearing of %f degrees", distToWP, (bearingToWP  * 180/pi + 360) % 360)

def main():
    rospy.init_node('gps_2d_nav_goal', anonymous=True)
    rospy.loginfo("Initiating GPS 2D Nav Goal Node.")

    while not rospy.is_shutdown(): #While ros comms are running smoothly
        rospy.Subscriber("waypoint", WayPoint, waypointSubscriber) #Subscribe to "pose", "fix" and "waypoint" ROS topics
        rospy.Subscriber("gps/fix", NavSatFix, gpsSubscriber)
        rospy.Subscriber("localization/filtered/pose_global", PoseWithCovarianceStamped, robotPoseSubscriber)
        rospy.spin()
        
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
