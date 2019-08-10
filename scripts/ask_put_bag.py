#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Int32
from abstract_module import AbstractModule
import time


class HmcAskPutBag(AbstractModule):
    def __init__(self):
        super(HmcAskPutBag, self).__init__(node_name="hmc_ask_put_bag")

        self.arm_pub = rospy.Publisher('/arm/control', Int32, queue_size=10)

        rospy.Subscriber("/hmc_nlp/function", String, self.execute_function)

    def execute_function(self, command):
        # type: (String) -> None
        """
        hmc_nlpから実行命令がpublishされたかを確認する
        :param: command: 実行する関数名
        :return: なし
        """
        if command.data == "ask_put_bag":
            self.print_node(command.data)
            self.ask_put_bag()

    def ask_put_bag(self):
        # type: () -> None
        """
        バッグをアームに置いたかを確認する
        :return: なし
        """
        self.arm_pub.publish(1)
        self.speak("Please put your bag in my hand.")
        time.sleep(5)
        speak_text = "Did you put your bag?"
        self.speak(speak_text)
        self.nlp_pub.publish(speak_text)


if __name__ == "__main__":
    HmcAskPutBag()
    rospy.spin()
