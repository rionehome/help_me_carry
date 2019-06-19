#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from sound_system.srv import NLPService


def callback_place(data):
	# pub_place.publish(data.data)
	rospy.wait_for_service('/sound_system/nlp')
	result = None
	try:
		result = rospy.ServiceProxy('/sound_system/nlp', NLPService)("Please go to " + data.data)
	except rospy.ServiceException, e:
		print "Service call failed"
		print e

	if result.response == '':
		print "None answer from navigation"

	else:
		print result.response
		pub_reach.publish()


def callback_reach(data):
	pub_reach.publish(True)


rospy.init_node("hmc_navigation_main", anonymous=True)
# pub_place = rospy.Publisher("hmc_navigation_main/place", String, queue_size=10) # 目的地の文字列を送る
pub_reach = rospy.Publisher("hmc_navigation_main/reach", Bool, queue_size=10)  # 目的地に着いたという情報を送る
rospy.Subscriber("help_me_carry/send_place", String, callback_place)  # 目的地の文字列を受け取る
rospy.Subscriber("/navigation/goal", Bool, callback_reach)  # 目的地に着いたという情報を受けとる
rospy.spin()
