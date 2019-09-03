#!/usr/bin/env python

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages

import rospy
import time
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import numpy as np
from std_srvs.srv import Empty

import cv2
from cv_bridge import CvBridge

from moveit_msgs.msg import MoveItErrorCodes
import final_msg_srv.srv
import final_msg_srv.msg

from final_msg_srv.srv import DesiredObject

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name


class SphericalService(object):
        def __init__(self):
                rospy.loginfo("Starting Spherical Grab Service")
                self.pick_type = PickAruco()
                rospy.loginfo("Finished SphericalService constructor")
                self.place_gui = rospy.Service("/place_gui", Empty, self.start_aruco_place)
                self.power_gui = rospy.Service("/power_gui", Empty, self.start_aruco_power)
                self.pick_gui = rospy.Service("/pick_gui", Empty, self.start_aruco_pick)
                self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
                self.home_gui  = rospy.Service("/home_gui", Empty, self.start_aruco_home)


        def start_aruco_pick(self, req):

                rospy.loginfo("Unfold arm safely")
                pmg = PlayMotionGoal()
                pmg.motion_name = 'pregrasp_weight'
                pmg.skip_planning = False
                self.play_m_as.send_goal_and_wait(pmg)
                rospy.loginfo("Done.")

                # rospy.loginfo("Start look around.")
                # pmg = PlayMotionGoal()
                # pmg.motion_name = 'head_look_around'
                # pmg.skip_planning = False
                # self.play_m_as.send_goal_and_wait(pmg)
                # rospy.loginfo("Look around have been done.")

                self.pick_type.pick_aruco("pick")
                return {}

        def start_aruco_power(self, req):
                self.pick_type.pick_aruco("power")
                return {}

        def start_aruco_place(self, req):
                self.pick_type.pick_aruco("place")
                return {}

        def start_aruco_home(self, req):
                rospy.loginfo("Go back to home")
                pmg = PlayMotionGoal()
                pmg.motion_name = 'home'
                pmg.skip_planning = False
                self.play_m_as.send_goal_and_wait(pmg)
                rospy.loginfo("Done.")

class PickAruco(object):
        def __init__(self):
                rospy.loginfo("Initalizing...")
                self.bridge = CvBridge()
                self.tfBuffer = tf2_ros.Buffer()
                self.tf_l = tf2_ros.TransformListener(self.tfBuffer)

                rospy.loginfo("Waiting for /pickup_pose AS...")
                self.pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction)
                time.sleep(1.0)
                if not self.pick_as.wait_for_server(rospy.Duration(20)):
                        rospy.logerr("Could not connect to /pickup_pose AS")
                        exit()
                rospy.loginfo("Waiting for /place_pose AS...")
                self.place_as = SimpleActionClient('/place_pose', PickUpPoseAction)

                self.place_as.wait_for_server()

                rospy.loginfo("Setting publishers to torso and head controller...")
                self.torso_cmd = rospy.Publisher(
                        '/torso_controller/command', JointTrajectory, queue_size=1)
                self.head_cmd = rospy.Publisher(
                        '/head_controller/command', JointTrajectory, queue_size=1)
                self.gripper_cmd = rospy.Publisher(
                        '/gripper_controller/command', JointTrajectory, queue_size=1)

                self.detected_pose_pub = rospy.Publisher('/detected_aruco_pose',
                                                         PoseStamped,
                                                         queue_size=1,
                                                         latch=True)

                rospy.loginfo("Waiting for '/play_motion' AS...")
                self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
                if not self.play_m_as.wait_for_server(rospy.Duration(20)):
                        rospy.logerr("Could not connect to /play_motion AS")
                        exit()
                rospy.loginfo("Connected!")
                rospy.sleep(1.0)
                rospy.loginfo("Done initializing PickAruco.")

                self.test_call = PoseStamped()
                self.ifreceive = False

                self.pub_done = rospy.Publisher("/process_done", final_msg_srv.msg.done, queue_size=10)
                self.pub_ready = rospy.Publisher("/pick_process_ready",final_msg_srv.msg.done_pick, queue_size=10)
                self.grasp = rospy.Service('/move_arm', final_msg_srv.srv.desired3D_pos, self.receive_pose)
                # self.client_place = rospy.ServiceProxy('say_something', DesiredObject)
                self.pub_pick_fail = rospy.Publisher("failed_to_pick", final_msg_srv.msg.done, queue_size=10)
                self.pick_fail = final_msg_srv.msg.done()

        def strip_leading_slash(self, s):
                return s[1:] if s.startswith("/") else s

        def pick_aruco(self, string_operation):
                # self.prepare_robot()

                rospy.sleep(2.0)
                rospy.loginfo("spherical_grasp_gui: Waiting for an aruco detection")
#                aruco_pose = rospy.wait_for_message('/aruco_single/pose', PoseStamped)

#                test_pose = rospy.Publisher('/Mypose',PoseStamped, queue_size=100)

#                aruco_pose = rospy.wait_for_messgae('/Point3D',PostStamped)

                #aruco_pose = rospy.wait_for_message('/Mypose', PoseStamped)

                while self.ifreceive == False:
                        rospy.loginfo("Waiting Info")
                        pick_ready = final_msg_srv.msg.done_pick()
                        pick_ready.done_pick = True
                        self.pub_ready.publish(pick_ready)
                        rospy.sleep(0.1)
                self.ifreceive = False

                pick_ready = final_msg_srv.msg.done_pick()
                pick_ready.done_pick = False
                self.pub_ready.publish(pick_ready)

                rospy.loginfo("Mypose has been recevied.")
                aruco_pose = self.test_call

                aruco_pose.header.frame_id = self.strip_leading_slash(aruco_pose.header.frame_id)
                rospy.loginfo("Got: " + str(aruco_pose))


                rospy.loginfo("spherical_grasp_gui: Transforming from frame: " +
                        aruco_pose.header.frame_id + " to 'base_footprint'")
                ps = PoseStamped()
                ps.pose.position = aruco_pose.pose.position
                ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
                ps.header.frame_id = aruco_pose.header.frame_id
                transform_ok = False
                while not transform_ok and not rospy.is_shutdown():
                        try:
                                transform = self.tfBuffer.lookup_transform("base_footprint",
                                                                           ps.header.frame_id,
                                                                           rospy.Time(0))
                                aruco_ps = do_transform_pose(ps, transform)
                                transform_ok = True
                        except tf2_ros.ExtrapolationException as e:
                                rospy.logwarn(
                                        "Exception on transforming point... trying again \n(" +
                                        str(e) + ")")
                                rospy.sleep(0.01)
                                ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint",aruco_pose.header.frame_id)
                        pick_g = PickUpPoseGoal()

                if string_operation == "pick":

                        rospy.loginfo("Setting cube pose based on ArUco detection")
                        pick_g.object_pose.pose.position = aruco_ps.pose.position
                        pick_g.object_pose.pose.position.z -= 0.1*(1.0/2.0)

                        rospy.loginfo("aruco pose in base_footprint:" + str(pick_g))

                        pick_g.object_pose.header.frame_id = 'base_footprint'
                        pick_g.object_pose.pose.orientation.w = 1.0
                        self.detected_pose_pub.publish(pick_g.object_pose)
                        rospy.loginfo("Gonna pick:" + str(pick_g))
                        self.pick_as.send_goal_and_wait(pick_g)
                        rospy.loginfo("Done!")

                        result = self.pick_as.get_result()
                        if str(moveit_error_dict[result.error_code]) != "SUCCESS":
                                rospy.logerr("Failed to pick, not trying further")
                                self.pick_fail.done = True
                                self.pub_pick_fail.publish(self.pick_fail)

                                return

                        # Move torso to its maximum height
#                        self.lift_torso()

                        # Raise arm
                        self.postgrasp_place()


                        # ======================= def new function to pick it otherplace else=============================
                         #Raise arm
                        #rospy.loginfo("Unfold arm safely")
                        # pmg = PlayMotionGoal()
                        # pmg.motion_name = 'pregrasp'
                        # pmg.skip_planning = False
                        # self.play_m_as.send_goal_and_wait(pmg)
                        # rospy.loginfo("Done.")

                        self.safe_place()

                        pick_msg = final_msg_srv.msg.done()
                        pick_msg.done = True
                        self.pub_done.publish(pick_msg)
                        rospy.loginfo("Pick part has done. Navigation begins")

                if string_operation == "power":
                        self.postgrasp_place()

                        # ======================= def new function to pick it otherplace else=============================
                if string_operation == "place":
                        # #rospy.lofinfo("Gonna move the object to home place")
                        # #rospy.loginfo("Gonna place near where it was")
                        # #pick_g.object_pose.pose.position.x = 1.00
                        # #pick_g.object_pose.pose.position.y = 1.00
                        # #pick_g.object_pose.pose.position.z = 1.00
                        # #self.place_as.send_goal_and_wait(pick_g)
                        # rospy.loginfo("Done!")
                        #
                        # # Place the object back to its position
                        # #rospy.loginfo("Gonna place near where it was")
                        # #pick_g.object_pose.pose.position.z += 0.05
                        # #self.place_as.send_goal_and_wait(pick_g)
                        # #rospy.loginfo("Done!")
                        # rospy.loginfo("Setting cube pose based on ArUco detection")
                        # pick_g.object_pose.pose.position = aruco_ps.pose.position
                        # pick_g.object_pose.pose.position.z -= 0.1 * (1.0 / 2.0)
                        #
                        # rospy.loginfo("aruco pose in base_footprint:" + str(pick_g))
                        #
                        # pick_g.object_pose.header.frame_id = 'base_footprint'
                        # pick_g.object_pose.pose.orientation.z += 0.05
                        # pick_g.object_pose.pose.orientation.w = 1.0
                        # self.detected_pose_pub.publish(pick_g.object_pose)
                        # rospy.loginfo("Gonna pick:" + str(pick_g))
                        # self.place_as.send_goal_and_wait(pick_g)
                        # rospy.loginfo("Done!")
                        #
                        # result = self.place_as.get_result()
                        # if str(moveit_error_dict[result.error_code]) != "SUCCESS":
                        #         rospy.logerr("Failed to pick, not trying further")
                        #         return
                        #
                        # # Move torso to its maximum height
                        # #                        self.lift_torso()

                        # # Raise arm
                        # self.look_around()
                        # self.postgrasp_place()
                        # self.safe_place()

                        self.open_hand()
			time.sleep(5)
			self.safe_place()

        def lift_torso(self):
                rospy.loginfo("Moving torso up")
                jt = JointTrajectory()
                jt.joint_names = ['torso_lift_joint']
                jtp = JointTrajectoryPoint()
                jtp.positions = [0.34]
                jtp.time_from_start = rospy.Duration(2.5)
                jt.points.append(jtp)
                self.torso_cmd.publish(jt)

        def lower_head(self):
                rospy.loginfo("Moving head down")
                jt = JointTrajectory()
                jt.joint_names = ['head_1_joint', 'head_2_joint']
                jtp = JointTrajectoryPoint()
                jtp.positions = [0.0, -0.75]
                jtp.time_from_start = rospy.Duration(2.0)
                jt.points.append(jtp)
                self.head_cmd.publish(jt)
                rospy.loginfo("Done.")

        def prepare_robot(self):
                rospy.loginfo("Unfold arm safely")
                pmg = PlayMotionGoal()
                pmg.motion_name = 'pregrasp_weight'
                pmg.skip_planning = False
                self.play_m_as.send_goal_and_wait(pmg)
                rospy.loginfo("Done.")

                self.lower_head()

                rospy.loginfo("Robot prepared.")

        def postgrasp_place(self):
                rospy.loginfo("Moving arm to post grasp pose")
                pmg = PlayMotionGoal()
                pmg.motion_name = 'pregrasp_weight'
                pmg.skip_planning = False
                self.play_m_as.send_goal_and_wait(pmg)
                rospy.loginfo("Raise object done.")

        def safe_place(self):
                rospy.loginfo("Moving arm to a safe pose")
                pmg = PlayMotionGoal()
                pmg.motion_name = 'home'
                pmg.skip_planning = False
                self.play_m_as.send_goal_and_wait(pmg)
                rospy.loginfo("Raise object done.")

        def look_around(self):
                rospy.loginfo("Start look around.")
                pmg = PlayMotionGoal()
                pmg.motion_name = 'head_look_around'
                pmg.skip_planning = False
                self.play_m_as.send_goal_and_wait(pmg)
                rospy.loginfo("Look around have been done.")


        def open_hand(self):
                rospy.loginfo("Moving head down")
                jt = JointTrajectory()
                jt.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
                jtp = JointTrajectoryPoint()
                jtp.positions = [0.04, 0.04]
                jtp.time_from_start = rospy.Duration(2.0)
                jt.points.append(jtp)
                self.gripper_cmd.publish(jt)
                rospy.loginfo("Done.")



        def receive_pose(self,req):
                test_msg = PoseStamped()
                test_msg.header.frame_id = 'base_footprint'
                test_msg.pose.position.x = req.x + 0.06
                test_msg.pose.position.y = req.y - 0.03
                # test_msg.pose.position.z = req.z + 0.075/2 - 0.075 * 1/8
                test_msg.pose.position.z = req.z
                test_msg.pose.orientation.w = 1.0
                self.test_call = test_msg

                self.ifreceive = True
                return final_msg_srv.srv.desired3D_posResponse(1)



if __name__ == '__main__':
        rospy.init_node('pick_aruco_demo')
        sphere = SphericalService()
        rospy.spin()


