#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.center_x = 0
        self.center_y = 0

        self.twist = Twist()

        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('video_window')

        cv2.namedWindow('iterations_image')
        self.iterations_dilate = 1
        self.iterations_erode = 1
        cv2.createTrackbar('iterations_dilate', 'iterations_image', 0, 15, self.set_iterations_dilate)
        cv2.createTrackbar('iterations_erode', 'iterations_image', 0, 15, self.set_iterations_erode)

    def set_iterations_dilate(self, val):
        ''' A callback function to change the dilation iterations '''
        self.iterations_dilate = val

    def set_iterations_erode(self, val):
        ''' A callback function to change the eroding iterations '''
        self.iterations_erode = val

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    
    def set_vals(self, speed=0, spin=0):
        """Helper function designed to take in a speed and angular velocity and set the appropriate values
        in the twist for later publishing."""
        self.twist.linear.x = speed; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = spin

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        kernel = np.ones((5,5),np.uint8)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                binary_image = cv2.inRange(self.cv_image, (0, 64, 0), (16, 255, 16))
                binary_image = cv2.erode(binary_image, kernel, iterations=self.iterations_erode)
                binary_image = cv2.dilate(binary_image, kernel, iterations=self.iterations_dilate)
                
                moments = cv2.moments(binary_image)
                if moments['m00'] != 0:
                    self.center_x = moments['m10']/moments['m00']
                    self.center_y = moments['m01']/moments['m00']
                    self.proportional_center = (self.center_x/640)-.5 #range from -.5 to .5
                    self.set_vals(speed=.5, spin=(-7.5*self.proportional_center))
                        
                else:
                    self.set_vals(spin=.2)


                print self.twist


                cv2.imshow('video_window', binary_image)
                cv2.waitKey(5)

            self.pub.publish(self.twist)
            # start out not issuing any motor commands
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()