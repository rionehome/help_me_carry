#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from abstract_module import AbstractModule


class HmcStopFollowMe(AbstractModule):
    def __init__(self):
        super(HmcStopFollowMe, self).__init__(node_name="hmc_stop_follow_me")
        rospy.init_node("hmc_stop_follow_me")

        self.follow_me_pub = rospy.Publisher("/follow_me/control", String, queue_size=10)
        self.nlp_pub = rospy.Publisher("/natural_language_processing/speak_sentence", String, queue_size=10)

        rospy.Subscriber("/hmc_nlp/function", String, self.execute_function)

    def execute_function(self, command):
        # type: () -> None
        """
        hmc_nlpから実行命令がpublishされたかを確認する
        :param: command: String
        :return: なし
        """
        if command.data == "stop_follow_me":
            self.stop_follow_me()

    def stop_follow_me(self):
        # type: () -> None
        """
        follow meを実行する
        :return: なし
        """
        self.follow_me_pub.publish('stop')
        self.speak_text = "May I help you"
        self.speak(self.speak_text)
        self.nlp_pub.publish(self.speak_text)


if __name__ == "__main__":
    HmcStopFollowMe()
    rospy.spin()
