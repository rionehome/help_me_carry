#!/usr/bin/env python
# -*- coding: utf-8 -*-
from hmc_start_node.msg import Activate
import rospy
from sound_system.srv import NLPService
from std_msgs.msg import String, Bool
import get_word
import os
import time
from pocketsphinx import get_model_path


class Help:
    def send_place_msg(self, place):  # 場所を伝えて次のノードにバトンタッチ
        # navigationに場所を伝える
        rospy.wait_for_service('/sound_system/nlp', timeout=1)
        response = rospy.ServiceProxy('/sound_system/nlp', NLPService)('Please go to {}'.format(place))
        print response.response
        if "OK" in response.response:
            navigation_wait = True
            while navigation_wait:
                time.sleep(0.1)
        else:  # 動けないなどのメッセージが返ってくる（ほぼない）
            # 次のノードに処理を渡す
            self.activate = False
            self.next_pub.publish(Activate(id=2))
    
    def main(self, sentense):  # 最初の音声認識。場所を判断。
        if self.start_flag:
            print(sentense)
            
            word_list = get_word.main(sentense.decode('utf-8'))
            print('place:{}'.format(word_list[0]))
            self.target_place = word_list[0]
            self.start_speaking('I will take this bag to {}? Please answer with yes or no'.format(self.target_place))
            
            while not self.finish_speaking_flag:
                continue
            # self.start_resume.publish('yes_no')
            self.chenge_dict.publish("yes_no_sphinx.dict")
            self.chenge_gram.publish("yes_no_sphinx.gram")
            self.start_resume.publish(True)
    
    def yes_no_recognition(self, yes_or_no, target):  # yes or noを判断。場所を確認
        if self.put_flag:
            if yes_or_no == 'yes':
                self.start_speaking('OK, I take this bag to {}'.format(target))
                while not self.finish_speaking_flag:
                    continue
                self.send_place_msg(target)
                self.start_flag = False
                self.put_flag = False
                self.txt = ''
                return
            else:
                time.sleep(5)
                self.start_speaking('Did you put your bag?')
                while not self.finish_speaking_flag:
                    continue
                self.start_resume.publish(True)
                return
        if yes_or_no == 'yes':
            self.start_speaking('OK, I take this bag to {}'.format(target))
            while not self.finish_speaking_flag:
                continue
            self.start_speaking('Sorry, I have no arm. So, I want you to put your bag on plate.')
            while not self.finish_speaking_flag:
                continue
            self.put_flag = True
            time.sleep(5)
            self.start_speaking('Did you put your bag?')
            while not self.finish_speaking_flag:
                continue
            self.start_resume.publish(True)  # 録音
        else:
            if self.loop_count >= 2:
                # 場所情報をランダムに発話していく.
                place_list = ['bed', 'kitchen', 'car', 'living room']
                self.index = self.index + 1
                self.target_place = place_list[self.index % len(place_list)]
                
                self.start_speaking('Is it {} ?'.format(self.target_place))
                # self.start_resume.publish('yes_no')
                self.chenge_dict.publish("yes_no_sphinx.dict")
                self.chenge_gram.publish("yes_no_sphinx.gram")
                self.start_resume.publish(True)
            else:
                self.start_speaking('Sorry, please say again')
                self.chenge_dict.publish("take_sphinx.dict")
                self.chenge_gram.publish("take_sphinx.gram")
                # self.start_resume.publish('help')
                self.start_resume.publish(True)
                self.loop_count += 1
    
    def start_speaking(self, sentence):
        self.finish_speaking_flag = False
        print(sentence)
        self.speak.publish(sentence)
    
    ###############################################################################
    def start_speech(self, data):  # activateを受け取ったら処理を開始
        if data.id == 1:
            print("first_nlp")
            self.start_flag = True
            self.activate = True
            self.chenge_dict.publish("take_sphinx.dict")
            self.chenge_gram.publish("take_sphinx.gram")
            # self.start_resume.publish('help')
            self.start_resume.publish(True)
    
    def get_txt(self, sentence):  # 音声認識の結果を取得
        # self.start_resume.publish('stop')  # 音声認識を止める
        self.start_resume.publish(False)
        if self.activate == True and sentence != '':
            self.txt = sentence.data
            print(self.txt)
            if self.txt == 'yes' or self.txt == 'no' or "take" in self.txt:
                if self.txt == 'yes' or self.txt == 'no':
                    self.yes_no_recognition(self.txt, self.target_place)
                else:
                    self.main(self.txt)
            else:
                self.start_resume.publish(True)
    
    def finish_speaking(self, data):  # 発話終了合図を受ける
        if data.data:
            self.finish_speaking_flag = True
    
    def navigation_goal_callback(self, data):
        if self.activate:
            self.activate = False
            # 次のノードに処理を渡す
            self.next_pub.publish(Activate(id=2))
    
    ########################################################################################
    def __init__(self):
        rospy.init_node('help_me_nlp_first_half_help', anonymous=True)
        
        self.start_resume = rospy.Publisher('/sound_system/recognition_start', Bool, queue_size=10)  # 音声認識開始
        
        self.speak = rospy.Publisher('/hmc_follow_me_nlp/speak_sentence', String, queue_size=10)  # 発話
        
        self.next_pub = rospy.Publisher('/help_me_carry/activate', Activate, queue_size=10)  # 次のノードに渡す
        
        self.chenge_dict = rospy.Publisher('/sound_system/sphinx/dict', String, queue_size=10)
        self.chenge_gram = rospy.Publisher('/sound_system/sphinx/gram', String, queue_size=10)
        
        rospy.Subscriber('/help_me_carry/activate', Activate, self.start_speech)  # ノードを起動
        
        rospy.Subscriber('/sound_system/recognition_result', String, self.get_txt)  # 音声認識結果取得
        rospy.Subscriber('/hmc_follow_me_nlp/finish_speaking', Bool, self.finish_speaking)  # 発話終了
        
        rospy.Subscriber('/navigation/goal', Bool, self.navigation_goal_callback)
        
        self.loop_count = 0
        self.index = 0
        self.model_path = get_model_path()
        self.dic_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary')
        
        self.navigation_wait = False
        self.activate = False
        self.start_flag = False
        self.finish_speaking_flag = False
        self.put_flag = False
        self.txt = ''
        self.target_place = ''


if __name__ == '__main__':
    try:
        Help()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
