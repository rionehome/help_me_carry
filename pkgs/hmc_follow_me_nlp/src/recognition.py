#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 音声認識

import rospy
from std_msgs.msg import String, Bool
import os
import sys
from pocketsphinx import LiveSpeech
from hmc_start_node.msg import Activate

class Recognition:
    # 音声認識
    def resume(self):
        print('== START RECOGNITION ==')
        self.speech = LiveSpeech(
            verbose=False, sampling_rate=8000, buffer_size=2048, no_search=False, full_utt=False,
            hmm=os.path.join(self.model_path, 'en-us'),
            lm=False,
            dic=self.dic_path,
            jsgf=self.jsgf_path
        )

        for text in self.speech:
            score = text.confidence()
            if score > 0.1:
                text = str(text)
                # self.speech_recognition = False
                self.pause()
                self.pub.publish(text) # 音声認識の結果をpublish
                break
            else:
                print("**noise**")

    # 音声認識ストップ
    def pause(self):
        print('== STOP RECOGNITION ==')
        self.speech = LiveSpeech(no_search=True)

    def recognition(self):
        while 1:
            if self.speech_recognition == True:
                self.resume()
            elif self.speech_recognition == False:
                self.pause()

    ##########################################
    # 音声認識再開のメッセージを受け取る
    def control(self, data):
        self.speech_recognition = data.data

    def control3(self, data):
        if data.id == 0:
            print "follow_me_nlp"
            self.activate = True

    ##########################################

    def __init__(self):
        rospy.init_node('hmc_follow_me_nlp_recognition', anonymous=True)
        self.model_path = '/usr/local/lib/python2.7/dist-packages/pocketsphinx/model'  # 音響モデルのディレクトリの絶対パス
        self.dictionary_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary')  # 辞書のディレクトリの絶対パス
        self.dic_path = os.path.join(self.dictionary_path, 'follow_me_sphinx.dict')
        self.jsgf_path = os.path.join(self.dictionary_path, 'follow_me_sphinx.gram')
        
        rospy.Subscriber('/help_me_carry/activate', Activate, self.control3)  # ノード起動
        rospy.Subscriber('/hmc_follow_me_nlp/recognition_start', Bool, self.control)
        rospy.Subscriber('/hmc_follow_me_nlp/stop_recognition', String, self.control)
        
        self.pub = rospy.Publisher('/hmc_follow_me_nlp/recognition_result', String, queue_size=10)
        
        self.speech = None
        self.speech_recognition = False
        self.activate = False
        print('== STOP RECOGNITION ==')
        rospy.spin()

if __name__ == '__main__':
    Recognition()