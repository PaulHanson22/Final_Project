#!/usr/bin/env python3

import time
from djitellopy import tello
import rospy
import cv2
from cv_bridge import CvBridge
from Final_Project.msg import Flip 
from Final_Project.msg import State
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty


class Driver():


    def __init__(self):
        self.drone = tello.Tello()
        self.drone.connect(False)
        self.drone.streamon()
        rospy.init_node("driver", anonymous=True) #inits ros node

        self.bridge = CvBridge()

        self.state_pub = rospy.Publisher("tello/state", State, queue_size=10)
        self.front_feed_pub = rospy.Publisher("tello/camera", Image, queue_size=10)

        rospy.Subscriber("tello/vel", Twist, self.vel_callback)
        rospy.Subscriber("tello/land", Empty, self.land_callback)
        rospy.Subscriber("tello/takeoff", Empty, self.takeoff_callback)
        rospy.Subscriber("tello/flip", Flip, self.flip_callback)

        self.state_msg = State()

        self.drone.TIME_BTW_RC_CONTROL_COMMANDS = 0.1
        self.prev_vel = None


    def set_state(self):
        self.state_msg.battery = self.drone.get_battery()
        self.state_msg.height = self.drone.get_height()
        self.state_msg.time = self.drone.get_flight_time()
        self.state_msg.highest_temp = self.drone.get_highest_temperature()
        self.state_msg.is_flying = self.drone.is_flying
        
        self.state_pub.publish(self.state_msg)


    def set_feed(self):
        if(self.drone.stream_on):
            feed = self.drone.get_frame_read().frame
            cv2.imshow("feed", feed)
            cv2.waitKey(1)
            front_feed_msg = self.bridge.cv2_to_imgmsg(feed, "bgr8")
            self.front_feed_pub.publish(front_feed_msg)
    

    def vel_callback(self, vel):
        if(self.drone.is_flying == True):
            if (vel != self.prev_vel):
                self.drone.send_rc_control(int(vel.linear.y),int(vel.linear.x),int(vel.linear.z),int(vel.angular.z))
        self.prev_vel = vel


    def land_callback(self, data):
        if(self.drone.is_flying == True):
            self.drone.land()
            print("landed")
            self.drone.is_flying = False
 

    def takeoff_callback(self, data):
        if(self.drone.is_flying != True):
            self.drone.takeoff()
            self.drone.is_flying = True
        

    def flip_callback(self, flip):
        if(self.drone.is_flying == True):
            self.drone.flip(flip.direction)


if __name__ == "__main__":
    driver = Driver()
    driver.drone.send_rc_control(0,0,0,0)
    while(not rospy.is_shutdown()):
        try:
            driver.set_state()
            driver.set_feed()
        except rospy.ROSInterruptException:
            break
    driver.drone.streamoff()
    cv2.destroyAllWindows()




