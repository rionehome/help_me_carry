#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Int32
from abstract_module import AbstractModule


class HmcReleaseBag(AbstractModule):
    def __init__(self):
        super(HmcReleaseBag, self).__init__(node_name="hmc_release_bag")

        self.arm_pub = rospy.Publisher('/arm/control', Int32, queue_size=10)
        self.human_detection_pub = rospy.Publisher("/human_detection/start", String, queue_size=10)

        rospy.Subscriber("/hmc_nlp/function", String, self.execute_function)

    def execute_function(self, command):
        # type: (String) -> None
        """
        hmc_nlpから実行命令がpublishされたかを確認する
        :param: command: 実行する関数名
        :return: なし
        """
        if command.data == "release_bag":
            self.print_node(command.data)
            self.release_bag()

    def release_bag(self):
        # type: () -> None
        """
        目的地到着後,バッグを置く.
        :return: なし
        """
        self.speak('I put the bag')
        self.arm_pub.publish(3)
        rospy.sleep(6)
        self.arm_pub.publish(4)
        rospy.sleep(4)
        self.human_detection_pub.publish("start")


if __name__ == "__main__":
    HmcReleaseBag()
    rospy.spin()
