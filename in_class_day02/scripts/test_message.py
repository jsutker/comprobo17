#!/usr/bin/env python

""" Exploring basics of creating messages inside of a ROS node """

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import rospy

rospy.init_node('my_first_node')
pub = rospy.Publisher('/cool_point', PointStamped, queue_size=10)

r = rospy.Rate(2)
while not rospy.is_shutdown():
  point_stamped_message = PointStamped(header=Header(stamp=rospy.Time.now(), 
                                                     frame_id="odom"), 
                                       point=Point(x=1.0, y=2.0))
  pub.publish(point_stamped_message)
  r.sleep()

print "Node is finished!"