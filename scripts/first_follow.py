#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from location.srv import RegisterLocation
from std_msgs.msg import String

from abstract_module import AbstractModule


class FirstFollow(AbstractModule):

    def __init__(self):
        # クラス継承を行う
        super(FirstFollow, self).__init__(node_name="hmc_follow_me", activate_no=1)

        self.follow_pub = rospy.Publisher('/follow_me/control', String, queue_size=10)

    ########################################################
    #       この main 関数内が一連の処理の流れとなる           #
    ########################################################
    def main(self):
        """
        処理の流れの中心
        音声認識したテキストデータからそれぞれの処理の関数を呼び出す
        :return: なし
        """
        print("Node: Follow Me")

        while True:
            self.change_sphinx_param("follow_me")
            self.wait_hot_word()
            text = self.start_recognition()

            if text == "follow me":
                # follow me 開始の処理
                self.follow_me()

            elif text == "stop following me":
                # follow me 停止の処理
                self.stop_follow_me()

            elif text == "here is the car":
                # 車の場所登録の処理
                # この処理が終わったら次のノードを起動する
                self.here_is_the_car()
                self.next_node_activate()
                return

    ########################################################
    #               それぞれの分岐する処理                    #
    ########################################################
    def follow_me(self):
        """
        Follow meの開始の一連の処理
        :return: なし
        """
        self.speak("Should I start following you?")
        answer = self.yes_no_recognition()

        if self.is_yes(answer):
            # Follow me 開始
            self.speak("OK, I start follow you.")
            self.follow_pub.publish("start")

        else:
            "もう一度命令を発話するように言う"
            self.speak_command_again()

    def stop_follow_me(self):
        """
        Follow meの停止の一連の処理
        :return: なし
        """
        self.speak("Can I stop following you?")
        answer = self.yes_no_recognition()

        if self.is_yes(answer):
            # Follow me 停止
            self.speak("OK, I stop.")
            self.follow_pub.publish("stop")

        else:
            # もう一度命令を発話するように言う
            self.speak_command_again()

    def here_is_the_car(self):
        """
        車の場所登録の一連の処理
        :return: なし
        """
        self.speak("Is Here the car?, yes or no.")
        answer = self.yes_no_recognition()

        if self.is_yes(answer):
            "現在地を「car」で登録"
            self.speak("OK, Here is the car.")
            rospy.ServiceProxy("/navigation/register_current_location", RegisterLocation)("car")

        else:
            "もう一度命令を発話するように言う"
            self.speak_command_again()


if __name__ == "__main__":
    FirstFollow()
    rospy.spin()
