#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from final_msg_srv.srv import *
from final_msg_srv.msg import *


class GraspingClient:

    def __init__(self):
        rospy.Subscriber("point3D_from_main", Coordinates_3D, self.callback_1)
        rospy.Subscriber("pick_process_ready", done_pick, self.callback_2)
        self.point3d_message = Coordinates_3D
        self.done_msg = done_pick
        self.desired_x = 0
        self.desired_y = 0
        self.desired_z = 0

    def desired_3d_client(self, px, py, pz):
        rospy.wait_for_service('move_arm')
        try:
            arm_moving = rospy.ServiceProxy('move_arm', desired3D_pos)
            desired3D_posResponse = arm_moving(px, py, pz)
            return desired3D_posResponse.reply
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def callback_1(self, Coordinates_3D):
        self.point3d_message = Coordinates_3D
        self.desired_x = self.point3d_message.x
        self.desired_y = self.point3d_message.y
        self.desired_z = self.point3d_message.z
        print('point received')

    def callback_2(self, done_pick):
        self.done_msg = done_pick
        print('done received')


if __name__ == '__main__':

    rospy.init_node('python_grasping_client')
    gp = GraspingClient()

    while True:
        print(gp.desired_x)
        print(gp.desired_y)
        print(gp.desired_z)
        if gp.done_msg.done_pick == 1:
            rospy.loginfo('got the pose, sending the service...')
            gp.desired_3d_client(gp.desired_x, gp.desired_y, gp.desired_z)
            #gp.done_msg.done_pick = 0

