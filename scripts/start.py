#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from abstract_module import AbstractModule


class HmcStart(AbstractModule):
    def __init__(self):
        super(HmcStart, self).__init__(node_name="hmc_start")

        rospy.Subscriber("/hmc_nlp/function", String, self.execute_function)

    def execute_function(self, command):
        # type: (String) -> None
        """
        hmc_nlpから実行命令がpublishされたかを確認する
        :param: command: 実行する関数名
        :return: なし
        """
        if command.data == "start":
            self.print_node(command.data)
            self.start()

    def start(self):
        # type: () -> None
        """
        開始ノード
        :return: なし
        """
        speak_text = "Please say follow me."  # 発話文
        self.speak(speak_text)
        self.nlp_pub.publish(speak_text)


if __name__ == "__main__":
    HmcStart()
    rospy.spin()
