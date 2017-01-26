#!/usr/bin/env python

""" Print distance to object immediately in front of the robot """

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
import rospy

class FiniteSteak:
  def __init__(self):
    self.frontDist = 10
    self.bumped = 0
    rospy.init_node('dist_e_stop')
    self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)
    self.bump_sub = rospy.Subscriber('/bump', Bump, self.process_bump)
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
    self.go_forward()
    self.do_the_thing()
    print "Node is finished!"

  def process_scan(self, msg):
    self.frontDist = msg.ranges[0]

  def process_bump(self, msg):
    self.bumped = msg.leftFront + msg.rightFront + msg.leftSide + msg.rightSide

  def set_vals(self, speed=0, spin=0):
    self.twist.linear.x = speed; self.twist.linear.y = 0; self.twist.linear.z = 0
    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = spin
    self.pub.publish(self.twist)

  def stop_everything(self):
    self.set_vals()

  def go_forward(self):
    self.set_vals(speed=1)

  def go_backward(self):
    self.set_vals(speed=-1)

  def turn_left(self):
    self.set_vals(spin=1)

  def do_the_thing(self):
    r = rospy.Rate(1)
    rr = rospy.Rate(10)
    twist_start_time = 0

    while not rospy.is_shutdown():

      self.pub.publish(self.twist)

      if self.bumped:
        self.go_backward()
      elif (self.twist.linear.x==-1) and ((self.frontDist > 0.6) or (self.frontDist == 0)):
        self.turn_left()
        twist_start_time = rospy.Time.now()

      if self.twist.angular.z and (rospy.Time.now() - twist_start_time):
        twist_start_time = 0
        self.go_forward()

      rr.sleep()


estop = FiniteSteak()