#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 自然言語処理

import os

import rospy
from hmc_start_node.msg import Activate
from location.msg import Location
from location.srv import RequestLocation
from sound_system.srv import *
from std_msgs.msg import String, Bool


class FourthGoBack:

    def __init__(self):

        self.activate_no = 4

        rospy.init_node("hmc_carry", anonymous=True)

        self.activate_pub = rospy.Publisher("/help_me_carry/activate", Activate, queue_size=10)
        self.navigation_pub = rospy.Publisher("/navigation/move_command", Location, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm/control', String, queue_size=10)

        rospy.Subscriber('/navigation/goal', Bool, self.navigation_callback)

    ########################################################
    #       この main関数内が一連の処理の流れとなる        #
    ########################################################
    def main(self, command):
        """
        処理の流れの中心
        人間検出したあとから車に戻るまでの一連の処理
        音声認識したテキストデータからそれぞれの処理の関数を呼び出す
        :param: Activateから飛んできたメッセージ 人間検出成功ならsuccess 失敗ならfailed
        :return: なし
        """
        if command == "success":
            # 人を見つけているので対話が発生
            self.speak("Would you help carrying groceries into the house? Please answer yes or no.")
            answer = self.yes_no_recognition()

            if self.is_yes(answer):
                # 車まで人を誘導(ただ車に戻るだけ)
                self.speak("Thank you. I will guide you to the car. Please follow me.")
                self.go_back()
            else:
                # 尋ねる人を間違えたようなので、人間検出に処理を戻す
                self.node_activate(3)

        else:
            # 人間検出失敗したので何もせずに車まで戻る
            self.speak("I couldn't find it.")
            self.go_back()

    def navigation_callback(self, message):
        # type: (Bool) -> None
        """
        ナビゲーション終了と共に呼び出される
        ここでHelp_me_carryの処理はすべて終了
        :param message: 移動成功ならTrue、失敗ならFalse
        :return: なし
        """
        is_success = message.data

        if is_success:
            self.speak("Here is the car.")

        else:
            self.speak("Sorry, I could not move to the car.")

        # すべての処理が終わったのでノードをKillして終了
        os.system('rosnode kill help_me_carry')

    ########################################################
    #               それぞれの分岐する処理                 #
    ########################################################
    def go_back(self):
        # type: () -> None
        """
        車の場所に戻る処理
        :return: なし
        """
        location_name = "car"
        location = rospy.ServiceProxy("/navigation/request_location", RequestLocation)(location_name).location
        self.navigation_pub.publish(location)

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
            self.main(message.text)

    def next_node_activate(self):
        # type: () -> None
        """
        次のノード(現在のID+1)に起動コマンドを送る
        :return: なし
        """
        activate = Activate()
        activate.id = self.activate_no + 1
        self.activate_pub.publish(activate)

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
    FourthGoBack()
    rospy.spin()
