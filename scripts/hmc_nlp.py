#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool
from abstract_module import AbstractModule
import time


class HmcNlp(AbstractModule):
    def __init__(self):
        super(HmcNlp, self).__init__(node_name="hmc_nlp")

        self.place = ""

        self.execute_func_pub = rospy.Publisher("/hmc_nlp/function", String, queue_size=10)
        self.place_info_pub = rospy.Publisher("/hmc/control", String, queue_size=10)

        rospy.Subscriber("/natural_language_processing/function_argument", String, self.function_argument_callback)
        rospy.Subscriber("/navigation/goal", Bool, self.navigation_goal_callback)
        rospy.Subscriber("/human_detection/finish", Bool, self.human_detection_callback)

        #self.start()



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

    def start(self):
        # type:  () -> None
        """
        Activateの命令が飛んできた場合
        activate_callbackから、このmain関数が呼び出される
        :return:なし
        """
        # 変数の初期化
        time.sleep(3)
        self.execute_func_pub.publish("start")

    def follow_me(self):
        # type: () -> None
        """
        follow meを実行する
        :return: なし
        """
        self.execute_func_pub.publish("follow_me")

    def stop_follow_me(self):
        # type: () -> None
        """
        follow meを止める
        :return: なし
        """
        self.execute_func_pub.publish("stop_follow_me")

    def ask_put_bag(self, place):
        # type: (String) -> None
        """
        バッグを置いたかどうかを確認する
        :param place: 場所名
        :return: なし
        """
        self.place = place  # 場所情報の保存
        self.execute_func_pub.publish("ask_put_bag")

    def restart_ask_put_bag(self):
        # type: () -> None
        """
        バッグを置いたかを聞いた際にNoが帰ってきた場合
        相手に再度、置いたかどうかを尋ねる
        :return: なし
        """
        self.execute_func_pub.publish("restart_ask_put_bag")

    def send_place_msg(self):
        # type: () -> None
        """
        bagを運ぶ先の場所情報をpublishする
        :return: なし
        """
        self.execute_func_pub.publish(self.place)

    def release_bag(self):
        # type: () -> None
        """
        ヒューマンディテクション時にバッグを地面に置く
        :return: なし
        """
        self.execute_func_pub.publish("release_bag")

    def back_to_the_car(self):
        # type: () -> None
        """
        human_detection後に車に戻る
        :return: なし
        """
        self.execute_func_pub.publish("back_to_the_car")

    def navigation_goal_callback(self, data):
        # type: (Bool) -> None
        """
        目的地に着いたことをsubscribeする
        :param data: 目的地に着いたらTrueが返る
        :return: なし
        """
        if self.place is not "car":
            self.release_bag()
        else:
            self.speak("Here is the car.")

    def human_detection_callback(self, data):
        # type: (Bool) -> None
        """
        human_detectionが終了したことをsubscribeする
        :param data: human_detectionが成功したらTrue, 失敗ならFalse
        :return: なし
        """
        if data.data:
            self.back_to_the_car()


if __name__ == "__main__":
    HmcNlp()
    rospy.spin()
