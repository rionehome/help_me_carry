#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sound_system.srv import *
from std_msgs.msg import String

class AbstractModule(object):

    def __init__(self, node_name):
        # type: (str) -> None
        rospy.init_node(node_name)
        self.nlp_pub = rospy.Publisher("/natural_language_processing/speak_sentence", String, queue_size=10)

    @staticmethod
    def speak(text):
        # type: (str) -> None
        """
        Sound_System に対して発話を要求する
        発話が終了するまで待機
        :param text: 発話内容
        :return: なし
        """
        print text
        rospy.wait_for_service("/sound_system/speak")
        rospy.ServiceProxy("/sound_system/speak", StringService)(text)

    @staticmethod
    def print_node(name):
        # type: (str) -> None
        """
        ノード名を表示
        :return: なし
        """
        print("\n###########################################\n")
        print("     Node: {}".format(name))
        print("\n###########################################\n")