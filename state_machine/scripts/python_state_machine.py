#!/usr/bin/env python
# # -*- coding: utf-8 -*-

import rospy
import smach
import numpy as np
import smach_ros
import time
from std_srvs.srv import Empty
from final_msg_srv.srv import *
from final_msg_srv.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import *
import roslaunch
import roslaunch.parent

class WakeUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['started','sleep'],
                             input_keys=['init_prob_table', 'target_room', 'p_combine', 'p_inverse'],
                             output_keys=['init_prob_table', 'target_room', 'p_combine', 'p_inverse'])
        self.speech_message = String()

    def execute(self, userdata):

        rospy.loginfo('Executing state Wakeup')
        self.speech_message = rospy.wait_for_message("recognizer/output", String)
        if "wake" in self.speech_message.data:

            return 'started'
        else:
            return 'sleep'


class StartPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['seeing'],
                             input_keys=['init_prob_table', 'target_room', 'p_combine', 'p_inverse'],
                             output_keys=['init_prob_table', 'target_room', 'p_combine', 'p_inverse'])
        self.done_message = False
        rospy.Subscriber("point3D", Coordinates_3D, self.callback)
        rospy.Subscriber("gesture_done", done, self.done_callback)
        rospy.Subscriber("point_done", done, self.done_callback_1)
        self.point3d_message = Coordinates_3D()
        self.pub3d = rospy.Publisher("point3D_from_main", Coordinates_3D, queue_size=10)
        self.done_class_gesture = done()
        self.done_class_point = done()

    def saysomething_client(self, words):
        rospy.wait_for_service('say_something')
        try:
            say_something = rospy.ServiceProxy('say_something', DesiredObject)
            say_something(words)
        except rospy.ServiceException, e:
            print "service call failed: %s" % e

    def callback(self, Coordinates_3D):
        self.point3d_message = Coordinates_3D
        print('received point coordinates')

    def done_callback(self, done):
        self.done_class_gesture = done
        print('received done gesture')

    def done_callback_1(self, done):
        self.done_class_point = done
        print('received done point')

    def lift_torso_client(self):
        rospy.wait_for_service('desired_signal1')
        try:
            lift_torso = rospy.ServiceProxy('desired_signal1', Empty)
            lift_torso()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def gesture_detection_client(self):
        rospy.wait_for_service('detect_gesture')
        try:
            gesture_detect = rospy.ServiceProxy('detect_gesture', Empty)
            gesture_detect()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def point_3d_client(self):
        rospy.wait_for_service('desired_object')
        try:
            point3d = rospy.ServiceProxy('desired_object', Empty)
            point3d()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, userdata):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [
            "/home/tiago/ros/workspaces/darknet_ws/src/darknet_ros/darknet_ros/launch/darknet_ros.launch"])

        rospy.loginfo('Executing state: start_point, picking the book')
        self.lift_torso_client()
        rospy.loginfo('lifting torso...')
        self.saysomething_client("Please stand before me")
        self.gesture_detection_client()
        print('a')
        self.saysomething_client("start gesture detection")
        self.done_class_gesture.done = False
        while True:
            if self.done_class_gesture.done:
                break
        self.launch.start(auto_terminate=True)
        self.saysomething_client("gesture detection finished, please show me the cup")

        while True:
            self.point_3d_client()
            if self.done_class_point.done:
                self.done_class_point.done = False
                break
        self.launch.shutdown()

        self.saysomething_client("i got the 3-D point of the object, starting grasping")
        print(self.point3d_message)
        self.pub3d.publish(self.point3d_message)
        return 'seeing'


class WithPerception(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['pick', 'pick_failed'],
                             input_keys=['init_prob_table', 'target_room', 'p_combine', 'p_inverse'],
                             output_keys=['init_prob_table', 'target_room', 'p_combine', 'p_inverse'])
        rospy.Subscriber("failed_to_pick", done, self.pick_fail_callback)
        self.pick_fail_class = done()
        #self.pick_fail_class.done = False

    def pick_fail_callback(self, done):
        self.pick_fail_class = done
        rospy.loginfo('pick_done_received')

    def saysomething_client(self, words):
        rospy.wait_for_service('say_something')
        try:
            say_something = rospy.ServiceProxy('say_something', DesiredObject)
            say_something(words)
        except rospy.ServiceException, e:
            print "service call failed: %s" % e

    def prepare_grasp_client(self):
        rospy.wait_for_service('pick_gui')
        try:
            prepare_pick = rospy.ServiceProxy('pick_gui', Empty)
            prepare_pick()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def safe_place_client(self):
        rospy.wait_for_service('home_gui')
        try:
            safe_place = rospy.ServiceProxy('home_gui', Empty)
            safe_place()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('Executing the state WithPerception')
        self.prepare_grasp_client()

        #self.pick_fail_class = rospy.wait_for_message("failed_to_pick", done)
        rospy.sleep(3)
        if self.pick_fail_class.done == False:
            rospy.loginfo('I got the object, starting to navigation')
            userdata.init_prob_table = np.array([0.4, 0.3, 0.2, 0.1])
            print('#----------------------------------------------------------------------------------#')
            print('The probability table is: ', userdata.init_prob_table)
            print('#----------------------------------------------------------------------------------#')
            ind = np.argmax(userdata.init_prob_table)
            userdata.target_room = ind
            userdata.p_inverse = 1 - userdata.init_prob_table[userdata.target_room]
            userdata.p_combine = userdata.p_inverse
            userdata.init_prob_table[userdata.target_room] = 0

            return 'pick'

        elif self.pick_fail_class.done == True:
            self.saysomething_client("i failed to pick")
            self.safe_place_client()
            self.pick_fail_class.done = False
            return 'pick_failed'

class DontknowWhereIam(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['localized'],
                             input_keys=['init_prob_table', 'target_room', 'p_inverse', 'p_combine'],
                             output_keys=['target_room', 'init_prob_table', 'p_inverse', 'p_combine'])
        self.speech_class = String

    def localization_client(self):
        rospy.wait_for_service('do_localization')
        try:
            localization = rospy.ServiceProxy('do_localization', Empty)
            localization()
        except rospy.ServiceException, e:
            print "service call failed: %s" % e

    def saysomething_client(self, words):
        rospy.wait_for_service('say_something')
        try:
            say_something = rospy.ServiceProxy('say_something', DesiredObject)
            say_something(words)
        except rospy.ServiceException, e:
            print "service call failed: %s" % e

    def execute(self, userdata):
        #self.launch.start()
        time.sleep(3)
        rospy.loginfo('starting the localization')
        self.saysomething_client("I am going to do localization, please leave me alone")
        self.localization_client()

        while True:
            #
            self.saysomething_client("have I succeed")
            self.speech_class = rospy.wait_for_message("recognizer/output", String)
            rospy.loginfo('received speech message')
            if self.speech_class.data == "yes":

                self.saysomething_client("i got yes, but i am not sure, can you repeat")
                self.speech_class = rospy.wait_for_message("recognizer/output", String)
                if self.speech_class.data == "yes":

                    self.saysomething_client("localization finished")
                    break
                else:
                    self.saysomething_client("i got confusion about the result")

                    continue

            elif self.speech_class.data == "no":
                self.saysomething_client("i got no, but i am not sure, can you repeat")
                self.speech_class = rospy.wait_for_message("recognizer/output", String)
                if self.speech_class.data == "yes":
                    self.saysomething_client("i got confusion about result")
                    continue
                else:
                    self.saysomething_client("ok, restart the localization")
                    self.localization_client()
                continue
            else:
                continue

        return 'localized'


class WithBook(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['navigation'],
                             input_keys=['init_prob_table', 'target_room', 'p_inverse', 'p_combine'],
                             output_keys=['target_room', 'init_prob_table', 'p_inverse', 'p_combine'])

    def down_torso_client(self):
        rospy.wait_for_service('desired_signal3')
        try:
            down_torso = rospy.ServiceProxy('desired_signal3', Empty)
            down_torso()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def navigation_client(self, px, py, pz, ox, oy, oz, ow):
        rospy.wait_for_service('move_base')
        try:
            navigation = rospy.ServiceProxy('move_base', desired_position)
            desired_positionResponse = navigation(px, py, pz, ox, oy, oz, ow)
            return desired_positionResponse.reply
            
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def saysomething_client(self, words):
        rospy.wait_for_service('say_something')
        try:
            say_something = rospy.ServiceProxy('say_something', DesiredObject)
            say_something(words)
        except rospy.ServiceException, e:
            print "service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('Executing state WithBook')
        self.down_torso_client()
        print("a")
        #goal2_x = np.array([29.8034629822, 11.0573921204, -9.42604541779])
        #goal2_y = np.array([-15.3525676727, -1.79483890533, 8.25204753876])
        #goal2_z = np.array([0.424459762897, 0.498450610436, 0.514532121891])
        #goal2_w = np.array([0.905446801133, 0.866918098182, 0.857471104786])
        goal2_x = np.array([0.0738643705845, 1.19945049286, -0.688966631889, 0.452061980963])
        goal2_y = np.array([0.83939230442, 1.13146495819, -1.46793973446, -2.94358921051])
        goal2_z = np.array([-0.683850107029, -0.999998674311, -0.00358902403849, 0.740269930332])
        goal2_w = np.array([0.729622526459, 0.00162830498851, 0.999993559432, 0.672309772535])
        print(userdata.target_room)
        target = userdata.target_room

        if target == 0:
            name_of_room = "Professor Office"
        elif target == 1:
            name_of_room = "Kitchen"
        elif target == 2:
            name_of_room = "library"
        else:
            name_of_room = "Karinne s Office"

        #target_str = str(target)
        basic_words = "the target room is "
        speaking_words = basic_words + name_of_room
        self.saysomething_client("starting navigation")
        self.saysomething_client(speaking_words)
        #self.navigation_client(goal1_x[target], goal1_y[target], 0, 0, 0, goal1_z[target], goal1_w[target])
        self.navigation_client(goal2_x[target], goal2_y[target], 0, 0, 0, goal2_z[target], goal2_w[target])
        return 'navigation'


class WithBookInRoomX(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['without_prof', 'with_prof'],
                             input_keys=['target_room', 'init_prob_table', 'p_inverse', 'p_combine'],
                             output_keys=['target_room', 'init_prob_table', 'p_inverse', 'p_combine'])

        rospy.Subscriber("find_face", face_message, self.face_callback)
        self.face_class = face_message()
        self.face_class.result = 5

    def face_callback(self, face_message):
        self.face_class = face_message
        print('received done message')

    def saysomething_client(self, words):
        rospy.wait_for_service('say_something')
        try:
            say_something = rospy.ServiceProxy('say_something', DesiredObject)
            say_something(words)
        except rospy.ServiceException, e:
            print "service call failed: %s" % e

    def face_decection_client(self):
        rospy.wait_for_service('detect_face')
        try:
            face_detect = rospy.ServiceProxy('detect_face', Empty)
            face_detect()
        except rospy.ServiceException, e:
            print "service call failed: %s" % e

    def lift_torso_client_for_face(self):
        rospy.wait_for_service('desired_signal2')
        try:
            lift_torso = rospy.ServiceProxy('desired_signal2', Empty)
            lift_torso()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('Executing state WithBookInRoomX')
        self.lift_torso_client_for_face()
        self.saysomething_client("I am looking for professor, can you please let me see your face")
        time.sleep(5)

        self.face_decection_client()
        print("a")

        while True:

            if self.face_class.result == 2:
                self.saysomething_client("I cannot find a human in this room")
                self.face_class.result = 5
                y_n = 0
                break

            elif self.face_class.result == 1:
                self.saysomething_client("Hello, Professor. Here is a shin cup for you.")
                y_n = 1
                break
            elif self.face_class.result == 0:
                self.saysomething_client("you are not the professor, bye")
                y_n = 0
                self.face_class.result = 5
                break

        if y_n == 1:
            return 'with_prof'
        else:
            return 'without_prof'


class WithBookInRoomXWithProf(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['give'],
                             output_keys=['target_room', 'p_inverse', 'p_combine', 'init_prob_table'],
                             input_keys=['target_room', 'init_prob_table', 'p_inverse', 'p_combine']
                             )
        rospy.Subscriber("wrist_ft", WrenchStamped, self.torque_callback)
        self.torque_class = WrenchStamped()
        self.a = 0
    def torque_callback(self, data):
        self.torque_class = data

    def give_cup_client_1(self):
        rospy.wait_for_service('power_gui')
        try:
            power = rospy.ServiceProxy('power_gui', Empty)
            power()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


    def give_cup_client_2(self):
        rospy.wait_for_service('place_gui')
        try:
            place = rospy.ServiceProxy('place_gui', Empty)
            place()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


    def execute(self, userdata):
        rospy.loginfo('Executing state With Book in Room X with Professor')
        self.give_cup_client_1()
        while True:
            if self.torque_class.wrench.force.x > -5 or self.torque_class.wrench.force.y < -6:
                self.give_cup_client_2()
                break
            else:
                continue
        return 'give'


class WithBookInRoomXWithoutProf(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['update_prob_table'],
                             output_keys=['target_room', 'p_inverse', 'p_combine', 'init_prob_table'],
                             input_keys=['target_room', 'init_prob_table', 'p_inverse', 'p_combine'])
        self.room_table = np.array([])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state With Book In RoomX Without Professor')

        p_find = 1 * userdata.init_prob_table / userdata.p_combine
        userdata.target_room = np.argmax(p_find)
        print('#----------------------------------------------------------------------------------#')
        print('The probability table is: ', p_find)
        print('#----------------------------------------------------------------------------------#')
        userdata.p_inverse = 1 - p_find[userdata.target_room]
        userdata.p_combine = userdata.p_inverse * userdata.p_combine
        userdata.init_prob_table[userdata.target_room] = 0
        rospy.loginfo('Executing state With Book in RoomX without professor')
        return 'update_prob_table'


class WithoutBook(smach.State):

    def __init__(self):
        smach.State.__init__(self,outcomes=['done'])

    def saysomething_client(self, words):
        rospy.wait_for_service('say_something')
        try:
            say_something = rospy.ServiceProxy('say_something', DesiredObject)
            say_something(words)
        except rospy.ServiceException, e:
            print "service call failed: %s" % e

    def execute(self, userdata):
        rospy.loginfo('Executing state Without Book')
        self.saysomething_client("mission complete")
        return 'done'



