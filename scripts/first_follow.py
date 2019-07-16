#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 自然言語処理

import rospy
from sound_system.srv import *
from std_msgs.msg import String, Bool
from hmc_start_node.msg import Activate
import datetime
import os
import math
import treetaggerwrapper as ttw


class FirstFollow:

    def __init__(self):

        self.activate_no = 1

        rospy.init_node("hmc_follow_me_nlp_main", anonymous=True)
        rospy.Subscriber("/sound_system/recognition_result", String, self.callback)

        self.pub = rospy.Publisher("/follow_me/control", String, queue_size=10)  # **制御にFollow me 開始停止の合図**
        self.pub_speak = rospy.Publisher("/hmc_follow_me_nlp/speak_sentence", String, queue_size=10)

        self.pub_start = rospy.Publisher("/sound_system/recognition_start", Bool, queue_size=10)  # 音声認識開始
        self.activate_pub = rospy.Publisher("/help_me_carry/activate", Activate, queue_size=10)

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
    #       この main関数内が一連の処理の流れとなる          #
    ########################################################
    def main(self):
        """
        処理の流れの中心
        音声認識したテキストデータからそれぞれの処理の関数を呼び出す
        :return: なし
        """
        self.wait_hot_word()
        text = self.start_recognition()
        text = self.text_modify(text)
        while True:
            if text == "follow me":
                # follow me の処理
                self.follow_me()
            elif text == "stop following me":
                self.stop_follow_me()
            elif text == "here is the car":
                self.here_is_the_car()

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
        else:
            "もう一度命令を発話するように言う"
            self.speak_command_again()

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
        response = rospy.ServiceProxy("/sound_system/recognition", RecognitionService)()
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

    def is_yes(self, text):
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