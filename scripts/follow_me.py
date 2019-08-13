#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from abstract_module import AbstractModule


class HmcFollowMe(AbstractModule):
    def __init__(self):
        super(HmcFollowMe, self).__init__(node_name="hmc_follow_me")
        self.follow_me_pub = rospy.Publisher("/follow_me/control", String, queue_size=10)
        
        rospy.Subscriber("/natural_language_processing/follow_me", String, self.follow_me)
    
    def follow_me(self, argument):
        # type: (String) -> None
        """
        follow meを実行する
        :param: argument: 関数用の引数が格納されている.
        本関数では空である.
        :return: なし
        """
        self.print_node(argument.data)
        speak_text = "When you arrive target point, please say stop following me."
        self.speak(speak_text)
        self.follow_me_pub.publish('start')
        self.nlp_pub.publish(speak_text)


if __name__ == "__main__":
    HmcFollowMe()
    rospy.spin()
