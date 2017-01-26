#!/usr/bin/env python

""" Print distance to object immediately in front of the robot """

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import rospy

class DistEmergencyStop:
  def __init__(self):
    self.frontDist = 10
    rospy.init_node('dist_e_stop')
    self.sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
    self.twist.linear.x = 1
    self.do_the_thing()
    print "Node is finished!"

  def process_scan(self, msg):
    self.frontDist = msg.ranges[0]

  def do_the_thing(self):
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
      if self.frontDist < 0.6:
        self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
      self.pub.publish(self.twist)

      r.sleep()

estop = DistEmergencyStop()