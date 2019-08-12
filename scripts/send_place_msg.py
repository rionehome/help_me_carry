#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Int32
from location.srv import *
from abstract_module import AbstractModule


class HmcSendPlaceMsg(AbstractModule):
    def __init__(self):
        super(HmcSendPlaceMsg, self).__init__(node_name="hmc_send_place_msg")

        self.arm_pub = rospy.Publisher('/arm/control', Int32, queue_size=10)

        rospy.Subscriber("/natural_language_processing/send_place_msg", String, self.send_place_msg)

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
        rospy.wait_for_service('/location/request_location', timeout=1)
        rospy.ServiceProxy('/location/request_location', RequestLocation)(place)


if __name__ == "__main__":
    HmcSendPlaceMsg()
    rospy.spin()
