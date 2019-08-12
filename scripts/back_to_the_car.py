#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from abstract_module import AbstractModule
from sound_system.srv import NLPService


class HmcBackToTheCar(AbstractModule):
    def __init__(self):
        super(HmcBackToTheCar, self).__init__(node_name="hmc_back_to_the_car")

        rospy.Subscriber("/natural_language_processing/back_to_the_car", String, self.back_to_the_car)

    def back_to_the_car(self, command):
        # type: () -> None
        """
        human_detection終了後,協力者を連れて車に戻る
        :param: argument: 関数用の引数が格納されている.
        本関数では空である.
        :return: なし
        """
        self.speak("I want you to help carrying groceries into the house.")
        self.speak("Please follow me.")
        rospy.wait_for_service("/sound_system/nlp", timeout=1)
        response = rospy.ServiceProxy("/sound_system/nlp", NLPService)("Please go to car")


if __name__ == "__main__":
    HmcBackToTheCar()
    rospy.spin()
