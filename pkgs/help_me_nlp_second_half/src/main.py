#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Help me carry,　Help me carry 後半部分の自然言語処理
from location.msg import Location
import time

from hmc_start_node.msg import Activate
from location.srv import RequestLocation
import rospy
from sound_system.srv import NLPService
from std_msgs.msg import String, Bool
import datetime
import os


class Help_me_nlp_second_half:
    def send_place_msg(self, place):
        # navigationに場所を伝える
        rospy.wait_for_service('/sound_system/nlp', timeout=1)
        response = rospy.ServiceProxy('/sound_system/nlp', NLPService)('Please go to {}'.format(place))
        print response.response
        if "OK" in response.response:
            navigation_wait = True
            while navigation_wait:
                time.sleep(0.1)
        else:
            self.speak("Sorry, I could not  move to the car.")
            os.system('rosnode kill help_me_nlp_second_half_recognition')
            os.system('rosnode kill help_me_nlp_second_half_speak')
            os.system('rosnode kill help_me_nlp_second_half_main')

    # 発話してログファイルに書き込む関数
    def speak(self, sentence):
        self.log_file(sentence, "s")
        rospy.loginfo("robot spoke: %s", sentence)
        self.pub_speak.publish(sentence)
        self.wait()

    # ログファイルの書き込みの関数
    def log_file(self, sentence, judge):
        with open(self.log_file_name, "a") as f:
            if judge == "h":
                f.write(str(datetime.datetime.now()) + "\t" + "robot heard:" + sentence + "\n")
            elif judge == "s":
                f.write(str(datetime.datetime.now()) + "\t" + "robot spoke:" + sentence + "\n")

    ##########################################################################
    # 目的地の近くで、もう一人のオペレーターを見つけたとき(画像認識からメッセージがきたとき)
    def person_dictation_callback(self, message):
        if message.id == 3 and message.text == "success":
            self.node_activate = True
            self.speak("Would you help carrying groceries into the house? Please answer yes or no.")
            self.pub_start.publish(True)
        if message.id == 3 and message.text == "failed":
            self.node_activate = True
            self.speak("I couldn't find it.")

    # Yes Noの音声認識結果を受け取る (目的地近くでオペレーターを見つけたとき)
    def recognition_callback(self, data):
        answer = data.data
        self.log_file(answer, "h")
        rospy.loginfo("robot heard: %s", answer)

        # "yes"のとき
        if answer == 'yes':
            self.speak("Thank you. I will guide you to the car. Please follow me.")
            # self.pub_stop_recognition.publish("stop node")
            # 場所の送信 & 移動
            self.send_place_msg("car")
        # "yes以外"
        else:
            self.speak("OK. Thank you.")
            back = Activate()
            back.id = 2
            self.pub_find_person.publish(back)  # 人に手伝ってもらえることに失敗

    def reach_car_callback(self, message):
        if not self.node_activate:
            return
        if message.data:
            self.speak("Here is the car.")
        else:
            self.speak("Sorry, I could not  move to the car.")
        os.system('rosnode kill help_me_nlp_second_half_recognition')
        os.system('rosnode kill help_me_nlp_second_half_speak')
        os.system('rosnode kill help_me_nlp_second_half_main')

    # 発話の間待つ
    def control(self, message):
        self.speak_flag = message.data

    def wait(self):
        self.speak_flag = False
        while self.speak_flag != True:
            pass

    def __init__(self):
        rospy.init_node("help_me_nlp_second_half_main")
        rospy.Subscriber("/help_me_carry/activate", Activate, self.person_dictation_callback)  # 画像認識終了の合図 **画像**
        
        rospy.Subscriber('/hmc_follow_me_nlp/finish_speaking', Bool, self.control)  # 発話終了の合図
        
        rospy.Subscriber("/help_me_nlp_second_half/recognition_result", String, self.recognition_callback)  # 音声認識結果
        rospy.Subscriber("/navigation/goal", Bool, self.reach_car_callback)  # 車に着いた合図 **制御**

        self.pub_speak = rospy.Publisher('/hmc_follow_me_nlp/speak_sentence', String, queue_size=10)  # 発話する文章
        self.pub_start = rospy.Publisher('/help_me_nlp_second_half/recognition_start', Bool, queue_size=10)
        
        self.pub_find_person = rospy.Publisher("/help_me_carry/activate", Activate, queue_size=10)  # 手伝ってくれる人がいたかどうかの合図 **画像**
        #self.pub_stop_recognition = rospy.Publisher("/help_me_nlp_second_half/stop_recognition", String, queue_size=10)  # 音声認識のループを抜ける
        
        self.speak_flag = False
        self.log_file_name = "{}/log{}.txt".format(os.path.join(os.path.dirname(os.path.abspath(__file__)), "log"), datetime.datetime.now())
        self.node_activate = False


if __name__ == '__main__':
    Help_me_nlp_second_half()
    rospy.spin()
