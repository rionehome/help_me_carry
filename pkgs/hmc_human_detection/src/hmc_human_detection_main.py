#!/usr/bin/env python
# -*- coding: utf-8 -*-
from hmc_start_node.msg import Activate
import rospy
from std_msgs.msg import Bool


def callback_activate(data):
	if data.id == 2:
		pub_start.publish(True)


def callback_finish(data):
	next = Activate
	if data.data:
		next.id = 3
		next.text = "success"
	else:
		next.id = 3
		next.text = "failed"
	pub_next.publish(next)


rospy.init_node("hmc_navigation_main", anonymous=True)
pub_start = rospy.Publisher("/human_detection/start", Bool, queue_size=10)
pub_next = rospy.Publisher('/help_me_carry/activate', Activate, queue_size=10)
rospy.Subscriber("/help_me_carry/activate", Activate, callback_activate)
rospy.Subscriber("/human_detection/finish", Bool, callback_finish)
rospy.spin()
