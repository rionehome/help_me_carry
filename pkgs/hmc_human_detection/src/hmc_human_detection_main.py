#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool


def callback_reach(data):
	if data.data:
		pub_start.publish(True)


def callback_finish(data):

	pass


rospy.init_node("hmc_navigation_main", anonymous=True)
# pub_place = rospy.Publisher("hmc_navigation_main/place", String, queue_size=10) # 目的地の文字列を送る
pub_start = rospy.Publisher("/human_detection/start", Bool, queue_size=10)
rospy.Subscriber("hmc_navigation_main/reach", Bool, callback_reach)
rospy.Subscriber("/human_detection/finish", Bool, callback_finish)
rospy.spin()
