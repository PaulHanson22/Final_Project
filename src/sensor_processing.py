#!/usr/bin/env python3

import time
from djitellopy import tello
import rospy
import cv2
from cv_bridge import CvBridge
from Final_Project.msg import Flip 
from Final_Project.msg import Mode
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
import mediapipe as mp
import math



class Sensors():

    def __init__(self):

        rospy.init_node("sensor_processing", anonymous=True) #inits ros node
        self.bridge = CvBridge()

        self.camera_sub = rospy.Subscribe("tello/camera", Image, self.camera_callback)

        self.mode_pub = rospy.Publisher("tello/mode", Mode, queue_size = 5)
        self.pose_pub = rospy.Publisher("tello/pose", Pose2D, queue_size = 5)
        self.land_pub = rospy.Publisher("tello/land", Empty, queue_size = 5)
        self.takeoff_pub = rospy.Publisher("tello/takeoff", Empty, queue_size = 5)

        self.mode_msg = Mode()
        self.pose_msg = Pose2D()

        self.FOV = (82.6 * (math.pi/180))

        #mediapipe init
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands

    
    def set_hand_gesture(self,image,point_index):
        with self.mp_hands.Hands(model_complexity=0,min_detection_confidence=0.5,min_tracking_confidence=0.5) as hands:

            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image)


            # Draw the hand annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.multi_hand_landmarks:

                x = (results.multi_hand_landmarks[0].landmark[point_index].x) 
                y = (results.multi_hand_landmarks[0].landmark[point_index].y) 

                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(image,hand_landmarks,self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing_styles.get_default_hand_landmarks_style(),
                        self.mp_drawing_styles.get_default_hand_connections_style())
                image = cv2.circle(image,(int(x),int(y)),10,(235,206,135),10)
            cv2.imshow('MediaPipe Hands', image)
        return (x,y)



    def set_voice(self):
        NotImplementedError

    
    def set_pose(self,distance,x,y):
        self.pose_msg.x = distance
        self.pose_msg.y = y
        self.pose_msg.theta = x
        self.pose_pub.publish(self.pose_msg)


    def camera_callback(self,camera):
        feed = self.bridge.imgmsg_to_cv2(camera, desired_encoding='passthrough')
        x,y = self.set_hand_gesture(feed,8)
        self.set_pose(0,x,y)



if __name__ == "__main__":
    sensors = Sensors()
    while(not rospy.is_shutdown()):
        try:
            pass
        except rospy.ROSInterruptException:
            pass
