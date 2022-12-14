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
        self.pose2D_sub = rospy.Subscriber("tello/pose2D", Pose2D, self.pose2D_callback)

        self.flip_pub = rospy.Publisher("tello/flip", Flip, queue_size = 5)
        self.vel_pub = rospy.Publisher("tello/vel", Twist, queue_size = 5)
        self.land_pub = rospy.Publisher("tello/land", Empty, queue_size = 5)
        self.takeoff_pub = rospy.Publisher("tello/takeoff", Empty, queue_size = 5)

        self.flip_msg = Flip()
        self.vel_msg = Twist()
        self.mode_msg = Mode()
        self.pose2D_msg = Pose2D()
        self.camera_msg = Image()
        self.terminate = False
        self.velocity = 50


    def set_defaults(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.z = 0
        self.flip_msg.direction = ""
        self.target_detected = False


    def mode_callback(self, mode):
        self.mode = mode.mode
        self.command = mode.command
    
    
    def pose2D_callback(self, pose):
        self.delta_x = pose.x
        self.delta_y = pose.y
        self.delta_theta = pose.theta
        self.target_detected = True


    def find_target(self):
        if self.pose:
            y_velocity = 10
            self.twist_msg.linear.y = y_velocity
            yaw_velocity = 72
            self.twist_msg.angular.z = yaw_velocity
            self.vel_pub.publish(self.vel_msg)


    def follow(self):
        set_distance = 100
        self.find_target()
        if self.target_detected == True:
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
            self.set_defaults()
    

    def target(self):
        set_distance = 0
        self.find_target()
        if self.target_detected == True:
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
            self.set_defaults()
    

    def tricks(self):
        flipped = False
        if self.command == 'flip forward':
            self.flip_msg.direction = "f"
        elif self.command == 'flip backward':
            self.flip_msg.direction = "b"
        elif self.command == 'flip left':
            self.flip_msg.direction = "l"
        elif self.command == 'flip right':
            self.flip_msg.direction = "r"
        else:
            pass
        flipped = True
        self.flip_pub.publish(self.flip_msg)
        if flipped == True:
            self.flip_msg.direction = ""


    def simple_movement(self):
        t0 = rospy.Time.now().to_sec()
        t1 = rospy.Time.now().to_sec()
        while(t1 - t0 < 1):
            t1 = rospy.Time.now().to_sec()
        if self.command == 'forward':
            self.vel_msg.linear.y = self.velocity
        elif self.command == 'backward':
            self.vel_msg.linear.y = -self.velocity
        elif self.command == 'left':
            self.vel_msg.linear.x = -self.velocity
        elif self.command == 'right':
            self.vel_msg.linear.x = self.velocity
        elif self.command == 'up':
            self.vel_msg.linear.z = self.velocity
        elif self.command == 'down':
            self.vel_msg.linear.z = -self.velocity
        elif self.command == 'turn left':
            self.vel_msg.angular.z = self.velocity
        elif self.command == 'turn right':
            self.vel_msg.angular.z = -self.velocity
        elif self.command == 'turn 90 degrees left':
            self.vel_msg.angular.z = -self.velocity
        elif self.command == 'turn 90 degrees right':
            self.rotate_clockwise(90)
        elif self.command == 'turn 180 degrees left':
            self.rotate_counter_clockwise(180)
        elif self.command == 'turn 180 degrees right':
            self.rotate_clockwise(180)
        elif self.command == 'turn 360 degrees left':
            self.rotate_counter_clockwise(360)
        elif self.command == 'turn 360 degrees right':
            self.rotate_clockwise(360)
        else:
            pass
        self.vel_pub.publish(self.vel_msg)
        self.set_defaults()


    def geometric_movement(self):
        if self.command == 'circle':
            y_velocity = 20
            self.twist_msg.linear.y = y_velocity
            yaw_velocity = 72
            self.twist_msg.angular.z = yaw_velocity
            self.vel_pub.publish(self.vel_msg)
            time.sleep(5)
        elif self.command == 'square':
            self.vel_msg.linear.y = -30
            self.vel_pub.publish(self.vel_msg)
            time.sleep(1)
            self.vel_msg.linear.y = 0
            self.vel_msg.linear.x = 30
            self.vel_pub.publish(self.vel_msg)
            time.sleep(1)
            self.vel_msg.linear.x = 0
            self.vel_msg.linear.y = 30
            self.vel_pub.publish(self.vel_msg)
            time.sleep(1)
            self.vel_msg.linear.y = 0
            self.vel_msg.linear.x = -30
            self.vel_pub.publish(self.vel_msg)
            time.sleep(1)
            self.set_defaults()
        else:
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
            else:
                pass
            movement.set_defaults()
        except rospy.ROSInterruptException:
            pass
    pygame.quit()
