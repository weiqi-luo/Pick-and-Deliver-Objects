#!/usr/bin/env python

import sys
import cv2
import os
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append('/usr/local/python')

from openpose import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from final_msg_srv.srv import DesiredObject
from final_msg_srv.msg import done

import numpy as np
import threading
import time


class gesture_detector:
    
    def __init__(self):        
        self.bridge = CvBridge()                       
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_rect_color",Image,self.callback_img)
        self.do_srv = rospy.Service("/detect_gesture", Empty, self.callback_do)
        self.done_pub = rospy.Publisher("/gesture_done", done, queue_size=10)
        #self.say_client = rospy.ServiceProxy('/say_something', DesiredObject)
        self.msg = done(done=True)
        self.something = DesiredObject()
        self.img = np.array([])
        self.count = 0
        self.do = False
        self.hand_left_old = long()
        self.hand_right_old = long()
                             

    def callback_do(self,data):
        self.count = 0
        self.do = True
        return []
    
    
    def callback_img(self,data): 
        ## convert the ros-msg to opencv-mat
        if self.do:
            try:
                self.img = self.bridge.imgmsg_to_cv2(data,"bgr8")
            except CvBridgeError as e:
                print(e)
            
    #def speak(self):
        #rospy.wait_for_service('say_something')
        #self.something.x = "Thank you! Please don't move."
        #try:
            #self.say_client(self.something.x)
        #except rospy.ServiceException, e:
            #print ("Service call failed: ", e)       

            
    def thread1_job(self):
        ## spin the subscriber
        print('T1 start\n')
        rospy.spin()    

        
    def thread2_job(self):  
        ## initialize the openpose
        print('T2 start\n')        
        params = dict()
        params["logging_level"] = 3
        params["output_resolution"] = "-1x-1"
        params["net_resolution"] = "-1x368"
        params["model_pose"] = "COCO"
        params["alpha_pose"] = 0.6
        params["scale_gap"] = 0.3
        params["scale_number"] = 1
        params["render_threshold"] = 0.05
        params["num_gpu_start"] = 0
        params["disable_blending"] = False
        params["default_model_folder"] = "/home/tiago/openpose/models/"
        openpose = OpenPose(params)
        ## abstract the gesture
        while True:
            time.sleep(0.1)
            if self.img.size != 0:
                while self.do:
                    arr, output_image = openpose.forward(self.img, True)
                    ## show image window
                    cv2.imshow("detect_gesture", output_image)
                    cv2.waitKey(3)
                    if arr.size == 0:
                        arr = np.ones((1,18,3))
                    else:
                        if arr[0][4][1]!=0 and arr[0][7][1]!=0 : 
                            hand_left = arr[0][4][1]
                            hand_right = arr[0][7][1]
                            if self.count:
                                delta_hand_left = self.hand_left_old - hand_left
                                delta_hand_right = self.hand_right_old - hand_right
                                ## detect the gesture
                                if delta_hand_left > 30 or delta_hand_right > 30:
                                    print("I see you!")                            
                                    self.do = False
                                    ##self.speak()
                                    self.done_pub.publish(self.msg)
                                    cv2.destroyAllWindows()
                                print delta_hand_left, delta_hand_right
                            ## update the data                                        
                            self.hand_left_old = hand_left
                            self.hand_right_old = hand_right
                            self.count +=1

            
    def main(self,args):
        rospy.init_node('gesture')
        thread1 = threading.Thread(target=self.thread1_job, name='T1')
        thread2 = threading.Thread(target=self.thread2_job, name='T2')
        thread1.start()
        thread2.start()
        thread1.join()
    
        
if __name__ == '__main__':
    ic = gesture_detector()
    ic.main(sys.argv)
    
