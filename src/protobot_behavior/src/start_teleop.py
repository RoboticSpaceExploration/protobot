#!/usr/bin/env python

import rospy
import roslaunch
import subprocess

subprocess.Popen('roslaunch protobot_teleop protobot_teleop.launch', shell=True)
