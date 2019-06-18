#!/usr/bin/env python
# coding: UTF-8
import time

import rospy
from std_msgs.msg import Bool

if __name__ == '__main__':
	rospy.init_node("hmc_start_node")

	pub = rospy.Publisher('hmc_follow_me_nlp/recognition_start', Bool, queue_size=10)
	time.sleep(1)
	pub.publish(True)
