#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from abstract_module import AbstractModule
from sound_system.srv import NLPService


class HmcFollowMe(AbstractModule):
    def __init__(self):
        super(HmcFollowMe, self).__init__(node_name="hmc_follow_me")
        self.follow_me_pub = rospy.Publisher("/follow_me/control", String, queue_size=10)

        rospy.Subscriber("/hmc_nlp/function", String, self.execute_function)

    def execute_function(self, command):
        # type: (String) -> None
        """
        hmc_nlpから実行命令がpublishされたかを確認する
        :param: command: 実行する関数名
        :return: なし
        """
        if command.data == "follow_me":
            self.print_node(command.data)
            self.follow_me()
        elif "follow_me" in command.data:
            self.print_node(command.data)
            self.stop_follow_me()

    def follow_me(self):
        # type: () -> None
        """
        follow meを実行する
        :return: なし
        """
        speak_text = "When you arrive target point, please say stop following me."
        self.follow_me_pub.publish('start')
        self.speak(speak_text)
        self.nlp_pub.publish(speak_text)

    def stop_follow_me(self):
        # type: () -> None
        """
        follow meを実行する
        :return: なし
        """
        self.follow_me_pub.publish('stop')
        # locationに車の位置を記録
        rospy.wait_for_service('/sound_system/nlp', timeout=1)
        print rospy.ServiceProxy('/sound_system/nlp', NLPService)('Here is car')
        speak_text = "May I help you?"
        self.speak(speak_text)
        self.nlp_pub.publish(speak_text)


if __name__ == "__main__":
    HmcFollowMe()
    rospy.spin()
