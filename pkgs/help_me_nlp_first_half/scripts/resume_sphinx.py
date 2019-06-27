#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 音声認識

import rospy
from std_msgs.msg import String, Bool
import os
from pocketsphinx import LiveSpeech, get_model_path
import subprocess

class Recognition:
    # 音声認識
    def resume(self):
        subprocess.call('aplay -q --quiet {}'.format(self.PATH_beep_start), shell=True)

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
                self.pub.publish(text)  # 音声認識の結果をpublish
                break
            else:
                print("**noise**")

    # 音声認識ストップ
    def pause(self):
        print('== STOP RECOGNITION ==')
        self.speech = LiveSpeech(no_search=True)

    def recognition(self):
        while 1:
            if self.speech_recognition == 'help':
                self.dic_path = os.path.join(self.dictionary_path, 'take_sphinx.dict')
                self.jsgf_path = (os.path.join(self.dictionary_path, "take_sphinx.gram"))
                self.resume()
            elif self.speech_recognition == 'yes_no':
                self.dic_path = os.path.join(self.dictionary_path, 'yes_no_sphinx.dict')
                self.jsgf_path = (os.path.join(self.dictionary_path, "yes_no_sphinx.gram"))
                self.resume()
            elif self.speech_recognition == 'stop':
                self.pause()


    ##############################################################################
    # 音声認識再開のメッセージを受け取る
    def control(self, data):
        self.speech_recognition = data.data

    ##############################################################################

    def __init__(self):
        rospy.init_node('help_me_nlp_half_recog', anonymous=True)
        self.model_path = get_model_path()  # 音響モデルのディレクトリの絶対パス
        self.dictionary_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary')  # 辞書のディレクトリの絶対パス
        self.beep_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'beep')
        self.PATH_beep_start = os.path.join(self.beep_path, 'start.wav')
        
        rospy.Subscriber('/help_me_nlp_first/resume_sphinx', String, self.control)

        self.pub = rospy.Publisher('/help_me_nlp_first/recognition_txt', String, queue_size=10)
        
        self.speech_recognition = False  # ノードを立ち上げた時から音声認識が始まる 最初は音声認識を停止する場合はFalse
        self.speech = None

        print('== STOP RECOGNITION ==')
        self.recognition()


if __name__ == '__main__':
    Recognition()
