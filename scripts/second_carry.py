#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from location.msg import Location
from location.srv import RequestLocation
from std_msgs.msg import String, Bool

from abstract_module import AbstractModule


class SecondCarry(AbstractModule):

    def __init__(self):
        # クラス継承を行う
        super(SecondCarry, self).__init__(node_name="hmc_carry", activate_no=2)

        self.navigation_pub = rospy.Publisher("/navigation/move_command", Location, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm/control', String, queue_size=10)

        rospy.Subscriber('/navigation/goal', Bool, self.navigation_callback)

    ########################################################
    #       この main 関数内が一連の処理の流れとなる           #
    ########################################################
    def main(self):
        """
        処理の流れの中心
        take this bag to the ~ の開始の一連の処理
        音声認識したテキストデータからそれぞれの処理の関数を呼び出す
        :return: なし
        """
        print("Node: Take this bag")

        while True:
            self.change_sphinx_param("take")
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
        if self.is_activate:
            self.put_bag()
            self.next_node_activate()

    ########################################################
    #               それぞれの分岐する処理                    #
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
            self.speak("OK.")
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
                self.speak('OK, I take this bag to the {}'.format(location_name))

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


if __name__ == "__main__":
    SecondCarry()
    rospy.spin()
