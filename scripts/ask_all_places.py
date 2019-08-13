#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from abstract_module import AbstractModule


class HmcAskAllPlaces(AbstractModule):
    def __init__(self):
        super(HmcAskAllPlaces, self).__init__(node_name="hmc_ask_all_places")
        self.place_list = ["kitchen", "bed_room", "living_room"]
        self.index = 0
        
        self.place_info_pub = rospy.Publisher("/hmc/send_place_msg", String, queue_size=10)
        
        rospy.Subscriber("/natural_language_processing/ask_all_places", String, self.ask_all_places)
    
    def ask_all_places(self, command):
        # type: (String) -> None
        """
        場所を一つずつ尋ねる
        :param: command: ゴミ
        :return: なし
        """
        speak_text = "Is it "
        self.speak(speak_text + "{}".format(self.place_list[self.index]))
        self.place_info_pub.publish(self.place_list[self.index])
        self.nlp_pub.publish(speak_text)
        self.index = self.index + 1


if __name__ == "__main__":
    HmcAskAllPlaces()
    rospy.spin()
