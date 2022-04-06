#!/usr/bin/env python

import rospy
import math
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Vector3


def callback(msg):
   #--Stopped here--
   #x = msg.transform
   pub = rospy.Publisher('/gate_detection/pointer', Vector3)
   pointvector = Vector3()
   transform = msg.transforms
   vec1 = transform[0]
   vec2 = transform[1]
   vec3 = [vec1.translation.x - vec2.translation.x, vec1.translation.y - vec2.translation.y, vec1.translation.z - vec2.translation.z]
   vec3[0] = vec3[0]/2 + vec1.translation.x
   vec3[1] = vec3[1]/2 + vec1.translation.y
   vec3[2] = vec3[2]/2 + vec1.translation.z
   magnitude = math.sqrt(vec3[0] ** 2 + vec3[1] ** 2 + vec3[2] ** 2)
   for i in range(0,3):
      vec3[i] = vec3[i]/magnitude
   pointvector.x = vec3[0]
   pointvector.y = vec3[1]
   pointvector.z = vec3[2]
   pub.publish(pointvector)
   
   #todo add which vecs are to be used

def main():
    rospy.init_node('gate_detection')
    rospy.Subscriber("/fiducial", FiducialTransformArray, callback)
    rospy.spin()
    
if __name__ == '__main__':
    main()
