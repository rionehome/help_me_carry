#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 自然言語処理

import rospy
from sound_system.srv import *
from std_msgs.msg import String, Bool, Int32
from location.srv import RegisterLocation, RequestLocation
from location.msg import Location
from hmc_start_node.msg import Activate


class SecondCarry:

    def __init__(self):

        self.activate_no = 2

        rospy.init_node("hmc_follow_me", anonymous=True)

        self.activate_pub = rospy.Publisher("/help_me_carry/activate", Activate, queue_size=10)
        self.navigation_pub = rospy.Publisher("/navigation/move_command", Location, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm/control', String, queue_size=10)

        rospy.Subscriber('/navigation/goal', Bool, self.navigation_goal_callback)

    ########################################################
    #       この main関数内が一連の処理の流れとなる        #
    ########################################################
    def main(self):
        """
        処理の流れの中心
        take this bag to the ~ の開始の一連の処理
        音声認識したテキストデータからそれぞれの処理の関数を呼び出す
        :return: なし
        """
        while True:
            self.wait_hot_word()
            text = self.start_recognition()
            text = self.text_modify(text)

            if "take this bag to " in text:
                # follow me 開始の処理
                location_name = text.replace("take this bag to the ", "")
                is_correct_location = self.ask_place(location_name)
                if is_correct_location:
                    self.take_and_carry_bag(location_name)
                    return

    def navigation_callback(self, message):
        # type: (Bool) -> None
        """
        ナビゲーション終了と共に呼び出される
        次のカバンをその場に置いたら次のノードに移る
        :param message: 移動成功ならTrue、失敗ならFalse
        :return: なし
        """
        self.put_bag()
        self.next_node_activate()

    ########################################################
    #               それぞれの分岐する処理                 #
    ########################################################
    def ask_place(self, location_name):
        # type: (str) -> bool
        """
        聞き取った場所がバッグを持っていく場所かどうか尋ねる処理
        :param location_name: 目的地の名前
        :return: 合っていれば True、 間違っていれば False
        """
        self.speak("Can I take this bag to the {}? yes or no.".format(location_name))
        answer = self.yes_no_recognition()

        if self.is_yes(answer):
            self.speak("OK, I start follow you.")
            return True

        else:
            # もう一度命令を発話するように言う
            self.speak_command_again()
            return False

    def take_and_carry_bag(self, location_name):
        # type: (str) -> None
        """
        ロボットがカバンを受け取る一連の処理
        :param location_name: 目的地の名前
        :return: なし
        """
        self.speak("Please put your bag in my hand.")
        self.arm_pub.publish("open")
        rospy.sleep(1.0)

        while True:
            # ひたすらカバンを置いたか確認する
            rospy.sleep(2.0)

            self.speak("Did you put your bag? yes or no.")
            answer = self.yes_no_recognition()

            if self.is_yes(answer):
                # 荷物を受け取ったので、ナビゲーションに処理が移る
                self.arm_pub.publish("close")
                rospy.sleep(1.5)
                self.start_speaking('OK, I take this bag to the {}'.format(location_name))

                # 移動命令を飛ばす
                location = rospy.ServiceProxy("/navigation/request_location", RequestLocation)(location_name).location
                self.navigation_pub.publish(location)
                return

            else:
                # カバンを置くように催促する
                self.speak("Please put your bag in my hand.")

    def put_bag(self):
        """
        カバンをその場に置く処理
        :return: なし
        """
        self.speak("I put this bag.")
        self.arm_pub.publish("release")
        rospy.sleep(1.5)
        self.arm_pub.publish("default")
        rospy.sleep(1.5)

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
