#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 自然言語処理

import rospy
from sound_system.srv import *
from std_msgs.msg import String, Bool, Int32
from location.srv import RegisterLocation, RequestLocation
from location.msg import Location
from hmc_start_node.msg import Activate


class ThirdHumanDetect:

    def __init__(self):
        self.activate_no = 3

        rospy.init_node("hmc_human_detect", anonymous=True)

        rospy.Subscriber("/help_me_carry/activate", Activate, self.activate_callback)
        self.activate_pub = rospy.Publisher("/help_me_carry/activate", Activate, queue_size=10)

    ########################################################
    #       この main関数内が一連の処理の流れとなる        #
    ########################################################
    def main(self):
        # 現状、人間検出の処理がどうなるか不明なので一旦保留
        print("Node: Human Detection")

        self.speak("Skip human detection.")
        self.next_node_activate()

    ########################################################
    #               それぞれの分岐する処理                 #
    ########################################################

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
    #     ここからは、ちょっとした細かい処理の関数群       #
    ########################################################

    @staticmethod
    def speak(text):
        # type: (str) -> None
        rospy.ServiceProxy("/sound_system/speak", StringService)(text)


if __name__ == "__main__":
    ThirdHumanDetect()
    rospy.spin()
