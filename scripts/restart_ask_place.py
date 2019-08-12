#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from abstract_module import AbstractModule


class HmcRestartAskPlace(AbstractModule):
    def __init__(self):
        super(HmcRestartAskPlace, self).__init__(node_name="hmc_restart_ask_place")
        self.loop_count = 1

        rospy.Subscriber("/hmc_nlp/function", String, self.execute_function)

    def execute_function(self, command):
        # type: (String) -> None
        """
        hmc_nlpから実行命令がpublishされたかを確認する
        :param: command: 実行する関数名
        :return: なし
        """
        if command.data == "restart_ask_place":
            self.print_node(command.data)
            self.restart_ask_place()

    def restart_ask_place(self):
        # type: () -> None
        """
        バッグを運ぶ場所を相手に確認した際に,拒否された場合
        再度場所を言ってもらうように促す.
        3度拒否された場合,全ての場所を一つずつ確認する.
        :return: なし
        """
        print self.loop_count
        if self.loop_count == 4:
            speak_text = "I will say all places."
            self.speak(speak_text + " please answer with yes or no.")
            self.nlp_pub.publish(speak_text)
            self.loop_count = 1

        self.speak("Sorry, please say the place again.")
        speak_text = "May I help you?"
        self.speak(speak_text)
        self.nlp_pub.publish(speak_text)
        self.loop_count = self.loop_count + 1


if __name__ == "__main__":
    HmcRestartAskPlace()
    rospy.spin()
