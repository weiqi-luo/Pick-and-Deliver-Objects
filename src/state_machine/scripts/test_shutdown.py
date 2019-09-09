#!/usr/bin/env python
# # -*- coding: utf-8 -*-

import rospy
import roslaunch
import time

rospy.init_node('test', anonymous=True)
#rospy.on_shutdown(rospy.shutdown)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/tiago/ros/workspaces/darknet_ws/src/darknet_ros/darknet_ros/launch/darknet_ros.launch"])

launch.shutdown()
