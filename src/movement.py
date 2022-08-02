#!/usr/bin/env python3

from this import d
from xml.etree.ElementInclude import DEFAULT_MAX_INCLUSION_DEPTH
import pygame
from djitellopy import tello
from re import L
import rospy
from Final_Project.msg import Flip 
from Final_Project.msg import State
from Final_Project.msg import Mode
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
import sys
import time



pygame.init()
display = (1280, 720)
pygame.display.set_mode(display)

class Movement:

    def __init__(self):

        rospy.init_node("movement", anonymous=True) #inits ros node

        self.mode_sub = rospy.Subscriber("tello/mode", Mode, self.mode_callback)
        self.camera_sub = rospy.Subscriber("tello/camera", Image, self.camera_callback)
        self.pose2D_sub = rospy.Subscriber("tello/pose2D", Image, self.pose2D_callback)

        self.flip_pub = rospy.Publisher("tello/flip", Flip, queue_size = 5)
        self.vel_pub = rospy.Publisher("tello/vel", Twist, queue_size = 5)
        self.land_pub = rospy.Publisher("tello/land", Empty, queue_size = 5)
        self.takeoff_pub = rospy.Publisher("tello/takeoff", Empty, queue_size = 5)
        

        rospy.Subscriber("tello/state", State, self.state_callback)

        self.flip_msg = Flip()
        self.vel_msg = Twist()
        self.mode_msg = Mode()
        self.camera_msg = Image()
        self.terminate = False
        self.velocity = 50


    def set_defaults(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.z = 0
        self.flip_msg.direction = ""


    def mode_callback(self, mode):
        mode = mode.mode
        command = mode.command
    
    def pose2D_callback(self, pose):
        delta_x = pose.x
        delta_y = pose.y
        delta_theta = pose.theta


    def find_target(self):
        if target_detected == False:
            y_velocity = 10
            self.twist_msg.linear.y = y_velocity
            yaw_velocity = 72
            self.twist_msg.angular.z = yaw_velocity
            self.vel_pub.publish(self.twist_msg)

    def follow(self):
        set_distance = 100
        self.find_target()
        if self.target_found == True:
            if self.delta_theta < 0.5:
                self.vel_msg.angular.z = self.velocity
            elif self.delta_theta > 0.5:
                self.vel_msg.angular.z = -self.velocity
            elif self.delta_y < 0.5:
                self.vel_msg.linear.z = self.velocity
            elif self.delta_y > 0.5:
                self.vel_msg.linear.z = -self.velocity
            else:
                pass
            self.vel_pub.publish(self.vel_msg)
    

    def target(self):
        set_distance = 0
        self.find_target()
        if self.target_found == True:
            if self.delta_theta < 0.5:
                self.vel_msg.angular.z = 2 * self.velocity
            elif self.delta_theta > 0.5:
                self.vel_msg.angular.z = 2 * -self.velocity
            elif self.delta_y < 0.5:
                self.vel_msg.linear.z = 2 * self.velocity
            elif self.delta_y > 0.5:
                self.vel_msg.linear.z = 2 * -self.velocity
            else:
                pass
            self.vel_pub.publish(self.vel_msg)
    
    def tricks():
        pass

    def simple_movement():
        pass

    def geometric_movement():
        pass
        
if __name__ == "__main__":
    movement = Movement()
    t0 = rospy.Time.now().to_sec()
    movement.set_defaults()
    while(not rospy.is_shutdown()):
        try:
            if movement.mode == "follow":
                movement.follow()
            elif movement.mode ==  "target":
                movement.target
            elif movement.mode ==  "find_target":
                movement.find_target()
            elif movement.mode ==  "tricks":
                movement.tricks()
            elif movement.mode ==  "simple_movement":
                movement.simple_movement()
            elif movement.mode ==  "geometric_movement":
                movement.geomentric_movement()
            movement.set_defaults()
        except rospy.ROSInterruptException:
            pass
    pygame.quit()
