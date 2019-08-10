#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from abstract_module import AbstractModule
import time


class HmcRestartAskPutBag(AbstractModule):
    def __init__(self):
        super(HmcRestartAskPutBag, self).__init__(node_name="hmc_restart_ask_put_bag")

        rospy.Subscriber("/hmc_nlp/function", String, self.execute_function)

    def execute_function(self, command):
        # type: (String) -> None
        """
        hmc_nlpから実行命令がpublishされたかを確認する
        :param: command: 実行する関数名
        :return: なし
        """
        if command.data == "restart_ask_put_bag":
            self.print_node(command.data)
            self.restart_ask_put_bag()

    def restart_ask_put_bag(self):
        # type: () -> None
        """
        少し待った後に,再度バッグを置いたかを確認する.
        :return: なし
        """
        self.speak("OK.")
        time.sleep(3)
        speak_text = "Did you put your bag?"
        self.speak(speak_text)
        self.nlp_pub.publish(speak_text)


if __name__ == "__main__":
    HmcRestartAskPutBag()
    rospy.spin()
