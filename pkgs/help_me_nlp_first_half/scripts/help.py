#!/usr/bin/env python
# -*- coding: utf-8 -*-
from hmc_start_node.msg import Activate
from location.msg import Location
import rospy
from sound_system.srv import NLPService
from std_msgs.msg import String, Bool
import get_word
import os
import time
from pocketsphinx import LiveSpeech, get_model_path


class help:
    def send_place_msg(self, place):  # 場所を伝えて次のノードにバトンタッチ
        # navigationに場所を伝える
        rospy.wait_for_service('/sound_system/nlp', timeout=1)
        response = rospy.ServiceProxy('/sound_system/nlp', NLPService)('Please go to {}'.format(place))
        print response.response
        if "OK" in response.response:
            navigation_wait = True
            while navigation_wait:
                time.sleep(0.1)
        else:
            # 次のノードに処理を渡す
            next = Activate()
            next.id = 2
            self.next_pub.publish(next)

    def main(self, sentense):  # 最初の音声認識。場所を判断。
        if (self.start_flag != False):
            print(sentense)
            
            word_list = get_word.main(sentense.decode('utf-8'))
            print('place:{}'.format(word_list[0]))
            self.target_place = word_list[0]
            self.start_speaking('I will take this bag to {} OK?'.format(self.target_place))

            while (self.finish_speaking_flag != True):
                continue


    def yes_no_recognition(self, yes_or_no, target_place):
        if (yes_or_no == 'yes'):
            self.start_speaking('OK, I take this bag to {}'.format(target_place))
            while (self.finish_speaking_flag != True):
                continue
            self.start_speaking('Sorry, I have no arm. So, I want you to put your bag on plate.')
            while (self.finish_speaking_flag != True):
                continue

            # navigationに場所を伝え、移動終了まで処理をする
            self.send_place_msg(target_place)
            self.start_flag = False
            self.txt = ''
        else:
            # 場所情報をランダムに発話していく.
            place_list = ['bed', 'kitchen', 'car', 'living room']
            self.loop_count = (self.loop_count + 1)%len(place_list)
            self.target_place = place_list[self.loop_count]
            self.start_speaking('Is it {} ?'.format(self.target_place))

    def start_speaking(self, sentence):
        self.finish_speaking_flag = False
        if (sentence != ''):
            print(sentence)
            self.speak.publish(sentence)

    ###############################################################################
    def start_speech(self, data):  # activateを受け取ったら処理を開始
        if (data.id == 1):
            print("first_nlp")
            self.start_flag = True
            self.activate = True

    def get_yesno(self, sentence):
        global take_ans
        if (sentence != ''):
            take_ans = sentence.data
            print(take_ans)

        print('I\'m taking yes or no...')
        time.sleep(1)
        self.yes_no.publish(True)

    def get_txt(self, sentence):  # 音声認識の結果を取得
        if (sentence != '' and self.target_place != ''):
            self.txt = sentence.data
            if self.txt == 'yes' or self.txt == 'no':
                self.yes_no_recognition(self.txt, self.target_place)
            else:
                self.main(self.txt)

        time.sleep(1)
        self.start_resume.publish(False)  # 音声認識を止める

    def finish_speaking(self, data):
        if (data.data == True):
            self.finish_speaking_flag = True

    def navigation_goal_callback(self, data):
        if self.activate:
            # 次のノードに処理を渡す
            next = Activate()
            next.id = 2
            self.next_pub.publish(next)
            self.activate = False

    ########################################################################################
    def __init__(self):
        rospy.init_node('help_me_nlp_first_half_help', anonymous=True)
        
        self.start_resume = rospy.Publisher('txt_start', Bool, queue_size=10)
        self.yes_no = rospy.Publisher('yes_no/recognition_start', Bool, queue_size=10)  # yes_no取得開始
        self.speak = rospy.Publisher('help_me_nlp_second_half/speak_sentence', String, queue_size=10)  # 発話
        self.next_pub = rospy.Publisher('/help_me_carry/activate', Activate, queue_size=10)

        rospy.Subscriber('/help_me_carry/activate', Activate, self.start_speech)  # ノードを起動
        rospy.Subscriber('yes_no/recognition_result', String, self.get_yesno)  # yes_no
        rospy.Subscriber('recognition_txt', String, self.get_txt)  # 音声認識結果取得
        rospy.Subscriber('help_me_nlp_second_half/finish_speaking', Bool, self.finish_speaking)  # 発話終了
        rospy.Subscriber('/navigation/goal', Bool, self.navigation_goal_callback)

        self.loop_count = 0
        self.model_path = get_model_path()
        self.dic_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary')
        self.navigation_wait = False
        self.activate = False
        self.start_flag = False
        self.finish_speaking_flag = False
        self.txt = ''
        self.target_place = ''


if __name__ == '__main__':
    try:
        help()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
