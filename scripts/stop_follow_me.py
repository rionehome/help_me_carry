#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from std_msgs.msg import String
from location.srv import *
from move.msg import AmountAction

from abstract_module import AbstractModule
from module.rviz_marker import RvizMarker


class HmcFollowMe(AbstractModule):
    def __init__(self):
        super(HmcFollowMe, self).__init__(node_name="hmc_stop_follow_me")
        self.follow_me_pub = rospy.Publisher("/follow_me/control", String, queue_size=10)
        self.amount_client = actionlib.SimpleActionClient("/move/amount", AmountAction)
        self.marker = RvizMarker()
        
        rospy.Subscriber("/natural_language_processing/stop_follow_me", String, self.stop_follow_me)
    
    def stop_follow_me(self, argument):
        # type: (String) -> None
        """
        follow meを実行する
        :param: argument: 関数用の引数が格納されている.
        本関数では空である.
        :return: なし
        """
        self.print_node(argument.data)
        self.follow_me_pub.publish('stop')
        # locationに車の位置を記録
        try:
            rospy.wait_for_service('/location/register_current_location', timeout=1)
            rospy.ServiceProxy('/location/register_current_location', RegisterLocation)("car")
            print "登録しました。"
        except rospy.ROSException:
            print "Error, not find location node."
        speak_text = "May I help you?"
        self.speak(speak_text)
        self.nlp_pub.publish(speak_text)


if __name__ == "__main__":
    HmcFollowMe()
    rospy.spin()
