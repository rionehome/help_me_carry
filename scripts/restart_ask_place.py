#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from abstract_module import AbstractModule


class HmcRestartAskPlace(AbstractModule):
    def __init__(self):
        super(HmcRestartAskPlace, self).__init__(node_name="hmc_restart_ask_place")
        self.loop_count = 3

        rospy.Subscriber("/natural_language_processing/restart_ask_place", String, self.restart_ask_place)

    def restart_ask_place(self, command):
        # type: (String) -> None
        """
        バッグを運ぶ場所を相手に確認した際に,拒否された場合
        再度場所を言ってもらうように促す.
        3度拒否された場合,全ての場所を一つずつ確認する.
        :param: command: ゴミ
        :return: なし
        """
        print self.loop_count
        if self.loop_count == 4:
            speak_text = "I will say all places."
            self.speak(speak_text + " please answer with yes or no.")
            self.nlp_pub.publish(speak_text)
            self.loop_count = 1
            return

        self.speak("Sorry, please say the place again.")
        speak_text = "May I help you?"
        self.speak(speak_text)
        self.nlp_pub.publish(speak_text)
        self.loop_count = self.loop_count + 1


if __name__ == "__main__":
    HmcRestartAskPlace()
    rospy.spin()
