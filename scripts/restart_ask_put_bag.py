#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from abstract_module import AbstractModule
import time


class HmcRestartAskPutBag(AbstractModule):
    def __init__(self):
        super(HmcRestartAskPutBag, self).__init__(node_name="hmc_restart_ask_put_bag")
        
        rospy.Subscriber("/natural_language_processing/restart_ask_put_bag", String, self.restart_ask_put_bag)
    
    def restart_ask_put_bag(self, argument):
        # type: (String) -> None
        """
        少し待った後に,再度バッグを置いたかを確認する.
        :param: argument: 関数用の引数が格納されている.
        本関数では空である.
        :return: なし
        """
        self.print_node(argument.data)
        self.speak("OK.")
        time.sleep(3)
        speak_text = "Did you put your bag?"
        self.speak(speak_text)
        self.nlp_pub.publish(speak_text)


if __name__ == "__main__":
    HmcRestartAskPutBag()
    rospy.spin()
