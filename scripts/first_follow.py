#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 自然言語処理

import rospy
from sound_system.srv import *
from std_msgs.msg import String, Bool
from location.srv import RegisterLocation
from hmc_start_node.msg import Activate
import datetime
import os
import math
import treetaggerwrapper as ttw


class FirstFollow:

    def __init__(self):

        self.activate_no = 1

        rospy.init_node("hmc_follow_me", anonymous=True)

        self.activate_pub = rospy.Publisher("/help_me_carry/activate", Activate, queue_size=10)
        self.follow_pub = rospy.Publisher('/follow_me/control', String, queue_size=10)

    ########################################################
    #       この main関数内が一連の処理の流れとなる          #
    ########################################################
    def main(self):
        """
        処理の流れの中心
        音声認識したテキストデータからそれぞれの処理の関数を呼び出す
        :return: なし
        """
        while True:
            self.wait_hot_word()
            text = self.start_recognition()
            text = self.text_modify(text)
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
                break

    ########################################################
    #               それぞれの分岐する処理                  #
    ########################################################
    def follow_me(self):
        """
        Follow meの開始の一連の処理
        :return: なし
        """
        self.speak("Should I start following you?")
        answer = self.yes_no_recognition()
        if self.is_yes(answer):
            "Follow me 開始"
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
        self.speak("Should I end following you?")
        answer = self.yes_no_recognition()
        if self.is_yes(answer):
            "Follow me 停止"
            self.speak("OK, I end follow you.")
            self.follow_pub.publish("stop")
        else:
            "もう一度命令を発話するように言う"
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

    ########################################################
    #               Activate関係の処理                     #
    ########################################################
    def activate_callback(self, message):
        # type: (Activate) -> None
        """
        指定IDがこのノードのIDなら処理開始
        :param message: Activateメッセージ
        :return: なし
        """
        if self.activate_no == message.id:
            self.main()

    def next_node_activate(self):
        # type: () -> None
        """
        次のノード(現在のID+1)に起動コマンドを送る
        :return: なし
        """
        activate = Activate()
        activate.id = self.activate_no + 1
        self.activate_pub.publish(activate)

    ########################################################
    #         ここからは、ちょっとした細かい処理の関数群         #
    ########################################################
    @staticmethod
    def wait_hot_word():
        """
        Hotword「hey, ducker」が検知されるまで待機
        :return:
        """
        rospy.ServiceProxy("/hotword/detect", HotwordService)()

    @staticmethod
    def start_recognition():
        # type: () -> str
        """
        Sphinxに対して音声認識を要求し、結果を返す
        :return: 認識結果の文字列
        """
        response = rospy.ServiceProxy("/sound_system/recognition", StringService)()
        text = response.response
        return text

    def yes_no_recognition(self):
        # type: () -> str
        """
        Sphinxに対して音声認識を要求し、結果を返す
        ただし返答が Yes か Noの時のみ返す
        Yes か No が出るまで聞き直し続ける
        :return: 認識結果の文字列
        """
        while True:
            answer = self.start_recognition()
            if self.is_yes_or_no(answer):
                return answer
            else:
                self.speak("Please say, Yes or No.")
        return None

    @staticmethod
    def is_yes(text):
        # type: (str) -> bool
        """
        与えられたテキストがYesならTrue
        No ならFalseを返す
        :return: 文字列
        """
        if text == "yes":
            return True
        return False

    @staticmethod
    def text_modify(text):
        # type: (str) -> str
        """
        与えられた文字列 に以下の処理を加える

        1. 文字列内のアンダースコア( _ ) をすべて半角スペースに変換
        2. 文字列内のアルファベットをすべて小文字化
        :param text: 処理する文字列
        :return: 処理後の文字列
        """
        if text is not None:
            text = text.replace("_", "")
            text = text.lower()
        return text

    @staticmethod
    def speak(text):
        # type: (str) -> None
        rospy.ServiceProxy("/sound_system/speak", StringService)(text)

    def speak_command_again(self):
        # type: () -> None
        self.speak("OK. Please say the command again.")

    @staticmethod
    def is_yes_or_no(text):
        # type: (str) -> bool
        """
        与えられた文字列が Yes もしくは No かを判別する
        :param text: 処理する文字列
        :return: Yes,Noの時は True それ以外は False
        """
        if text == "yes" or text == "no":
            return True
        return False


if __name__ == "__main__":
    FirstFollow()
    rospy.spin()
