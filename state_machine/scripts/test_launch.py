#!/usr/bin/env python
# # -*- coding: utf-8 -*-

import rospy
import roslaunch
import time
from std_srvs.srv import *
from final_msg_srv.msg import *

class LaunchDarknet:

    def __init__(self):
#rospy.on_shutdown(rospy.shutdown)
        #self.do_srv = rospy.Service("/start_darknet", Empty, self.callback_do)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/tiago/ros/workspaces/darknet_ws/src/darknet_ros/darknet_ros/launch/darknet_ros.launch"])
        rospy.Subscriber("open_or_close", done, self.callback)
        self.do_class = done()


    def callback(self, done):
        self.do_class = done


    def run_darknet(self):

        if self.do_class.done:
            self.launch.start()
            time.sleep(10)
            #while True:
                #if self.do_class.done == False:
                    #break
                #time.sleep(1)



if __name__ == '__main__':
    rospy.init_node('launch_dark')
    dk = LaunchDarknet()

    #while True:
    dk.run_darknet()
    rospy.spin()

#import roslaunch

#cli_args = ['/home/tiago/ros/workspaces/darknet_ws/src/darknet_ros/darknet_ros/launch/darknet_ros.launch']

#roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0])]
#uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

#parent.start()
