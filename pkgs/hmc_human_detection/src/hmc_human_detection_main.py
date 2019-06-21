#!/usr/bin/env python
# -*- coding: utf-8 -*-
from hmc_start_node.msg import Activate
import rospy
from std_msgs.msg import String, Bool


def callback_activate(data):
	if data.id == 2:
		pub_start.publish(True)


def callback_finish(data):
	if data.data:
		next = Activate
		next.id = 3
		next.text = "success"



rospy.init_node("hmc_navigation_main", anonymous=True)
# pub_place = rospy.Publisher("hmc_navigation_main/place", String, queue_size=10) # 目的地の文字列を送る
pub_start = rospy.Publisher("/human_detection/start", Bool, queue_size=10)
rospy.Subscriber("/help_me_carry/activate", Activate, callback_activate)
rospy.Subscriber("/human_detection/finish", Bool, callback_finish)
rospy.spin()
