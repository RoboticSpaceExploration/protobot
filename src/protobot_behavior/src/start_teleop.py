#!/usr/bin/env python

import rospy
import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent (uuid, ['/home/roselab/protobot/src/protobot_teleop/launch/protobot_teleop.launch'])
launch.start()
