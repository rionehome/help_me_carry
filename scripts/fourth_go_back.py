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
from abstract_module import AbstractModule


class FourthGoBack(AbstractModule):

    def __init__(self):
        super(FourthGoBack, self).__init__(node_name="hmc_go_back", activate_no=4)

        self.navigation_pub = rospy.Publisher("/navigation/move_command", Location, queue_size=10)

        rospy.Subscriber('/navigation/goal', Bool, self.navigation_callback)

    ########################################################
    #       この main 関数内が一連の処理の流れとなる           #
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
            self.speak("I couldn't find the person.")
            self.go_back()

    def navigation_callback(self, message):
        # type: (Bool) -> None
        """
        ナビゲーション終了と共に呼び出される
        ここでHelp_me_carryの処理はすべて終了
        :param message: 移動成功ならTrue、失敗ならFalse
        :return: なし
        """
        if self.is_activate:
            is_success = message.data

            if is_success:
                self.speak("Here is the car.")

            else:
                self.speak("Sorry, I could not move to the car.")

            # すべての処理が終わったのでノードをKillして終了
            os.system('rosnode kill help_me_carry')

    ########################################################
    #               それぞれの分岐する処理                    #
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


if __name__ == "__main__":
    FourthGoBack()
    rospy.spin()
