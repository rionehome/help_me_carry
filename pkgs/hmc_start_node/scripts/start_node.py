#!/usr/bin/env python
# coding: UTF-8
import rospy
from std_msgs.msg import String, Bool

if __name__ == '__main__':
	rospy.sleep(10)
	rospy.init_node("hmc_start_node")
	pub = rospy.Publisher('follow_me_nlp/recognition_start', Bool, queue_size=10)
	pub.publish(True)
