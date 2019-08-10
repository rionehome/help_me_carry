#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from abstract_module import AbstractModule
from sound_system.srv import NLPService


class HmcBackToTheCar(AbstractModule):
    def __init__(self):
        super(HmcBackToTheCar, self).__init__(node_name="hmc_back_to_the_car")

        rospy.Subscriber("/hmc_nlp/function", String, self.execute_function)

    def execute_function(self, command):
        # type: (String) -> None
        """
        hmc_nlpから実行命令がpublishされたかを確認する
        :param: command: 実行する関数名
        :return: なし
        """
        if command.data == "back_to_the_car":
            self.print_node(command.data)
            self.back_to_the_car()

    def back_to_the_car(self):
        # type: () -> None
        """
        human_detection終了後,協力者に説明し車に戻る
        :return: なし
        """
        self.speak("I want you to help carrying groceries into the house.")
        self.speak("Please follow me.")
        rospy.wait_for_service("/sound_system/nlp", timeout=1)
        response = rospy.ServiceProxy("/sound_system/nlp", NLPService)("Please go to car")


if __name__ == "__main__":
    HmcBackToTheCar()
    rospy.spin()
