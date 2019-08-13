#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Int32
from abstract_module import AbstractModule
import time


class HmcAskPutBag(AbstractModule):
    def __init__(self):
        super(HmcAskPutBag, self).__init__(node_name="hmc_ask_put_bag")

        self.arm_pub = rospy.Publisher("/arm/control", Int32, queue_size=10)
        self.place_info_pub = rospy.Publisher("/hmc/send_place_msg", String, queue_size=10)

        rospy.Subscriber("/natural_language_processing/ask_put_bag", String, self.ask_put_bag)

    def ask_put_bag(self, argument):
        # type: (String) -> None
        """
        バッグをアームに置いたかを確認する
        :param: argument: 関数用の引数が格納されている.
        本関数では空である.
        :return: なし
        """
        if argument.data is not "":
            self.place_info_pub.publish(argument.data)
        self.arm_pub.publish(1)
        self.speak("Please put your bag in my hand.")
        time.sleep(5)
        speak_text = "Did you put your bag?"
        self.speak(speak_text)
        self.nlp_pub.publish(speak_text)


if __name__ == "__main__":
    HmcAskPutBag()
    rospy.spin()
