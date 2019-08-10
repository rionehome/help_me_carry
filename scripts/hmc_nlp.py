#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from sound_system.srv import *
from abstract_module import AbstractModule

class HmcNlp(AbstractModule):
    def __init__(self):
        super(HmcNlp, self).__init__(node_name="hmc_nlp")

        rospy.init_node("hmc_nlp")

        self.execute_func_pub = rospy.Publisher("/hmc_nlp/function", String, queue_size=10)
        self.place_info_pub = rospy.Publisher("/hmc/control", String, queue_size=10)

        rospy.Subscriber("/natural_language_processing/function_argument", String, self.function_argument_callback)

        self.main()

    def main(self):
        # type:  () -> None
        """
        Activateの命令が飛んできた場合
        activate_callbackから、このmain関数が呼び出される
        :return:なし
        """
        # 変数の初期化
        self.speak_text = "Please say follow me"  # 発話文
        self.speak(self.speak_text)
        self.nlp_pub.publish(self.speak_text)

    def function_argument_callback(self, data):
        # type: (String) -> None
        """
        1.natural_language_processingから"関数名,引数名"を受け取る
        2.関数を実行する
        :param data:"関数名,引数名"
        :return: なし
        """
        function_argument_list = data.data.split(",", 1)
        # 引数が無いとき
        if function_argument_list[1] == "":
            eval("self." + function_argument_list[0])()
        else:
            eval("self." + function_argument_list[0])(function_argument_list[1])

    def follow_me(self):
        # type: () -> None
        """
        follow meを実行する
        :return: なし
        """
        print "follow me"
        self.execute_func_pub.publish("follow_me")

    def stop_follow_me(self):
        # type: () -> None
        """
        follow meを止める
        :return: なし
        """
        print "stop follow me"
        self.execute_func_pub.publish("stop_follow_me")

    def send_place_msg(self, place):
        # type: (String) -> None
        """
        bagを運ぶ先の場所情報をpublishする
        :param place: 場所名
        :return: なし
        """
        print "send_place_msg"
        print place
        self.execute_func_pub.publish(place)

if __name__ == "__main__":
    HmcNlp()
    rospy.spin()