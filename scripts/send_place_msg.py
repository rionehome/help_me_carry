#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Int32
from abstract_module import AbstractModule


class HmcSendPlaceMsg(AbstractModule):
    def __init__(self):
        super(HmcSendPlaceMsg, self).__init__(node_name="hmc_send_place_msg")

        self.place_list = ["kitchen", "car", "bed_room", "living_room"]

        self.place_info_pub = rospy.Publisher("/hmc/control", String, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm/control', Int32, queue_size=10)

        rospy.Subscriber("/hmc_nlp/function", String, self.execute_function)

    def execute_function(self, command):
        # type: (String) -> None
        """
        hmc_nlpから実行命令がpublishされたかを確認する
        :param: command: 実行する関数名
        :return: なし
        """
        if command.data in self.place_list:
            self.print_node(command.data)
            self.send_place_msg(command.data)

    def send_place_msg(self, place):
        # type: (String) -> None
        """
        bagを運ぶ先の場所情報をpublishする
        :param place: 場所名
        :return: なし
        """
        self.arm_pub.publish()
        self.speak("OK, I go to the {}".format(place))
        self.arm_pub.publish(2)
        rospy.sleep(3)
        self.place_info_pub.publish(place)


if __name__ == "__main__":
    HmcSendPlaceMsg()
    rospy.spin()
