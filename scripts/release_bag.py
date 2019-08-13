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
        
        rospy.Subscriber("/natural_language_processing/release_bag", String, self.release_bag)
    
    def release_bag(self, argument):
        # type: (String) -> None
        """
        目的地到着後,バッグを置く.
        :param: argument: 関数用の引数が格納されている.
        本関数では空である.
        :return: なし
        """
        self.print_node(argument.data)
        self.speak('I put the bag')
        self.arm_pub.publish(3)
        rospy.sleep(6)
        self.arm_pub.publish(4)
        rospy.sleep(4)
        self.human_detection_pub.publish("start")


if __name__ == "__main__":
    HmcReleaseBag()
    rospy.spin()
