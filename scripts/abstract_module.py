#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sound_system.srv import *
from hmc_start_node.msg import Activate
import sys


class AbstractModule(object):

    def __init__(self, node_name, activate_no):
        # type: (str, int) -> None

        self.is_activate = False

        rospy.init_node(node_name)
        self.activate_no = activate_no
        self.activate_pub = rospy.Publisher("/help_me_carry/activate", Activate, queue_size=10)

        rospy.Subscriber("/help_me_carry/activate", Activate, self.activate_callback)

    def main(self):
        """
        抽象メソッド

        Activateの命令が飛んできた場合 activate_callbackから、このmain関数が呼び出される
        よってこの関数をサブクラス(このクラスを継承したクラス)でオーバライドしておけば、そちらが呼び出される
        :return:
        """
        raise Exception("main関数がオーバーライドされていません")
        sys.exit(1)
        pass

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
            self.is_activate = True
            self.print_node()
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
        self.is_activate = False

    def node_activate(self, no):
        # type: () -> None
        """
        指定のノード番号に起動コマンドを送る
        :param: ID
        :return: なし
        """
        activate = Activate()
        activate.id = no
        self.activate_pub.publish(activate)
        self.is_activate = False

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
    def change_sphinx_param(text):
        # type: (str) -> None
        """
        Sphinxに対して辞書の切り替えを要求
        :param text: 切り替える辞書の名前
        :return:
        """
        rospy.ServiceProxy("/sound_system/sphinx/param", StringService)(text)

    def start_recognition(self):
        # type: () -> str
        """
        Sphinxに対して音声認識を要求し、結果を返す
        :return: 認識結果の文字列
        """
        response = rospy.ServiceProxy("/sound_system/recognition", StringService)()
        text = response.response
        text = self.text_modify(text)
        return text

    def yes_no_recognition(self):
        # type: () -> str
        """
        Sphinxに対して音声認識を要求し、結果を返す
        ただし返答が Yes か Noの時のみ返す
        Yes か No が出るまで聞き直し続ける
        :return: 認識結果の文字列
        """
        self.change_sphinx_param("yes_no")
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
            text = text.replace("_", " ")
            text = text.lower()

        return text

    @staticmethod
    def speak(text):
        # type: (str) -> None
        rospy.ServiceProxy("/sound_system/speak", StringService)(text)

    def speak_command_again(self):
        # type: () -> None
        self.speak("OK, Please say the command again.")

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

    def print_node(self):
        print("\n###########################################\n")
        print("     Node: {}".format(self.__class__.__name__))
        print("\n###########################################\n")
