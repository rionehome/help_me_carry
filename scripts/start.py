#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from abstract_module import AbstractModule


class HmcStart(AbstractModule):
    def __init__(self):
        super(HmcStart, self).__init__(node_name="hmc_start")

        rospy.Subscriber("/natural_language_processing/start", String, self.start)

    def start(self, argument):
        # type: (String) -> None
        """
        開始ノード
        :param: argument: 関数用の引数が格納されている.
        本関数では空である.
        :return: なし
        """
        speak_text = "Please say follow me."  # 発話文
        self.speak(speak_text)
        self.nlp_pub.publish(speak_text)


if __name__ == "__main__":
    HmcStart()
    rospy.spin()
