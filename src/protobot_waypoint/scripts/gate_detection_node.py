#!/usr/bin/env python

import rospy
from fiducial_msgs.msg import FiducialTransform

def callback(msg):
   #--Stopped here--
   #x = msg.transform
   

def main():
    rospy.init_node('gate_detection')
    rospy.Subscriber("/fiducial", FiducialTransform, callback)
    rospy.spin()
    
if __name__ == '__main__';
    main()
