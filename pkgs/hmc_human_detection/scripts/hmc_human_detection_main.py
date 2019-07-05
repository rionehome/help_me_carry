#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

from hmc_start_node.msg import Activate
import rospy
from std_msgs.msg import Bool, String


class HmcHumanDetection:
    
    def __init__(self):
        
        # HumanDetectionが全体から割り当てられたID
        self.no = 2
        
        rospy.init_node("hmc_navigation_main", anonymous=True)
        self.pub_start = rospy.Publisher("/human_detection/start", String, queue_size=10)
        self.pub_next = rospy.Publisher('/help_me_carry/activate', Activate, queue_size=10)
        rospy.Subscriber("/help_me_carry/activate", Activate, self.callback_activate)
        rospy.Subscriber("/human_detection/finish", Bool, self.callback_finish)
        
        rospy.spin()
    
    def callback_activate(self, data):
        # type: (Activate) -> None
        """
        ROS Subscriberコールバック関数
        Activateの通知を受け取り、IDが2の場合はHumanDetectionを起動

        :param data: Activate(int32, string)型のメッセージ
        :return: なし
        """
        if data.id == self.no:
            print("human_detection_nlp")
            #time.sleep(3)
            print("wait")
            # HumanDetectionのノードに開始の合図を送る
            self.pub_start.publish("start")
    
    def callback_finish(self, data):
        # type: (Bool) -> None
        """
        ROS Subscriberコールバック関数
        HumanDetectionのノードから終了の合図を受信
        次のIDのノードに起動メッセージを送る

        :param data: Bool型メッセージ
        :return: なし
        """
        activate = Activate()
        # 次のIDを指定
        activate.id = self.no + 1
        # HumanDetectionが成功したかどうかも次のノードに送る
        if data.data:
            activate.text = "success"
        else:
            activate.text = "failed"
        self.pub_next.publish(activate)


if __name__ == '__main__':
    HmcHumanDetection()
