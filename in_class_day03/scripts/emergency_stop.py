#!/usr/bin/env python

""" Print distance to object immediately in front of the robot """

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from neato_node.msg import Bump
import rospy

class EmergencyStop:
  def __init__(self):
    self.bumped = 0
    rospy.init_node('e_stop')
    self.sub = rospy.Subscriber('/bump', Bump, self.process_bump)
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
    self.twist.linear.x = 1
    self.do_the_thing()
    print "Node is finished!"

  def process_bump(self, msg):
    self.bumped = msg.leftFront + msg.rightFront + msg.leftSide + msg.rightSide

  def do_the_thing(self):
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
      if self.bumped:
        self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
      self.pub.publish(self.twist)

      r.sleep()

estop = EmergencyStop()