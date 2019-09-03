#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import face_recognition
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from final_msg_srv.msg import face_message
from final_msg_srv.srv import DesiredObject

class image_converter:
    
    def __init__(self):

        self.bridge = CvBridge()      
        self.do_srv = rospy.Service("/detect_face", Empty, self.callback_do)                 
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_rect_color", Image, self.callback)
        self.done_pub = rospy.Publisher("/find_face", face_message, queue_size=10)      
        self.msg = face_message()    
        self.do = False
        self.count_sum = 0        
        self.count = 0
        self.count_find = 0
        
        ## load the photo of professor and learn how to recognize it
        face1_image = face_recognition.load_image_file("/home/tiago/BavarianRobot/src/face_detection/scripts/professor.jpg")              
        face1_encoding = face_recognition.face_encodings(face1_image)[0]

        ## create arrays of known face encodings and their names
        self.known_faces_encodings = [face1_encoding]      
        self.known_faces_names = ["Professor Cheng"]
        
        
    def callback_do(self,data):
        self.count_sum = 0 
        self.count = 0
        self.count_find = 0
        self.do = True
        return []
        
        
    def callback(self,data):
        if self.do:
            if self.count_sum <80:
		self.count_sum +=1
		print(self.count_sum)
                if self.count < 10:
                    ## convert the ros-msg to opencv-mat in BGR(which opencv uses) or RGB color(which face_recognition uses)
                    try:
                        cv_image_bgr = self.bridge.imgmsg_to_cv2(data,"bgr8")
                        cv_image_rgb = self.bridge.imgmsg_to_cv2(data,"rgb8")
                    except CvBridgeError as e:
                        print(e)
                    
                    ## find all the faces  
                    face_locations = face_recognition.face_locations(cv_image_rgb)
                    face_encodings = face_recognition.face_encodings(cv_image_rgb, face_locations)
                    
                    ## loop through each face to find out professor
                    for (top,right,bottom,left), face_encoding in zip(face_locations,face_encodings):
                        matches = face_recognition.compare_faces(self.known_faces_encodings,face_encoding)
                        name = "Unkown"
                        self.count += 1
                        if True in matches:
                            ## get the name
                            first_match_index = matches.index(True)
                            name = self.known_faces_names[first_match_index]
                            self.count_find +=1
                        # mark the face of professor
                        cv2.rectangle(cv_image_bgr,(left, top),(right,bottom),(0,0,255),2)
                        cv2.rectangle(cv_image_bgr, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                        font = cv2.FONT_HERSHEY_DUPLEX
                        cv2.putText(cv_image_bgr, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
                    
                    ## show image window
                    cv2.imshow("detect_face", cv_image_bgr)
                    cv2.waitKey(3)
                
                ## publish the result
                if self.count == 10:
                    self.do = False               
                    ## find professor
                    if self.count_find >5:
                        self.msg.result = 1
                        self.done_pub.publish(self.msg)
                    ## find someone but not professor                    
                    else:
                        self.msg.result = 0
                        self.done_pub.publish(self.msg)                                     
                    cv2.destroyAllWindows()
                
                ## publish the result: find nobody    
                if self.count_sum == 80:
                    self.do = False 
                    self.msg.result = 2
                    self.done_pub.publish(self.msg)
                    cv2.destroyAllWindows()
                
    def speak(self):
        rospy.wait_for_service('say_something')
        try:
            self.say_client(self.something.x)
        except rospy.ServiceException, e:
            print ("Service call failed:", e)       
                                              

def main(args):
    
    ic = image_converter()
    rospy.init_node('face')  
    rospy.spin()
    
if __name__ == '__main__':
    main(sys.argv)
    
    
#self.say_client = rospy.ServiceProxy('/say_something', DesiredObject)
#self.something = DesiredObject()
#self.something.x = "Hello, Professor. Here is a potato chips for you."
#self.speak()
    
