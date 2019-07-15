#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 発話 + ビープ音

import rospy
from std_msgs.msg import String, Bool
import subprocess
import os

class Speak:
    def callback(self, data):
        speech = data.data
        # ビープ音のディレクトリの絶対パスを変数に設定
        # ビープ音 発話
        speech = speech.replace("_", " ")
        speech_wave = os.path.join(self.beep_path, 'speech.wav')
        subprocess.call(['pico2wave', '-w={}'.format(speech_wave), speech])
        subprocess.call('aplay -q --quiet {}'.format(speech_wave), shell=True)
        self.pub_finish.publish(True)

    def __init__(self):

        self.beep_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'beep')

        rospy.init_node('hmc_follow_me_nlp_speak', anonymous=True)
        rospy.Subscriber('/hmc_follow_me_nlp/speak_sentence', String, self.callback)
        self.pub_finish = rospy.Publisher('/hmc_follow_me_nlp/finish_speaking', Bool, queue_size=10)

if __name__ == '__main__':
    Speak()
    rospy.spin()

