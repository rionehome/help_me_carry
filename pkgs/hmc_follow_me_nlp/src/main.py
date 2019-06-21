#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 自然言語処理

import rospy
from std_msgs.msg import String, Bool
from hmc_start_node.msg import Activate
import time
import datetime
import os
import math
import treetaggerwrapper as ttw


class Follow_me_nlp:
	# 類似度を計算
	def get_similar(self, a, b):
		TTWDIR= os.path.join(os.path.expanduser('~'),'tree-tagger-install')
		tagger=ttw.TreeTagger(TAGLANG='en', TAGDIR=TTWDIR)
		dic2 = {}
		tags2 = tagger.TagText(a.decode("utf-8"))
		for tag2 in tags2:
			tag2 = tag2.split('\t')
			if tag2[2] not in dic2:
				dic2[tag2[2]] = 1
			else:
				dic2[tag2[2]] += 1

		dic = {}
		tags = tagger.TagText(b.decode("utf-8"))
		for tag in tags:
			tag = tag.split('\t')
			if tag[2] not in dic:
				dic[tag[2]] = 1
			else:
				dic[tag[2]] += 1

		length_d = 0.0
		for d in dic:
			length_d += dic[d]
		length_d = math.sqrt(length_d)

		length_d2 = 0.0
		for d2 in dic2:
			length_d2 += dic2[d2]
		length_d2 = math.sqrt(length_d2)

		score = 0.0
		cos = 0.0
		for d in dic:
			if d in dic2:
				score += dic[d] * dic2[d]
		if score != 0.0:
			cos = score / (length_d * length_d2)
		return cos

	# 発話してログファイルに書き込むの関数
	def speak(self, sentence):
		self.log_file_spoke(sentence)
		rospy.loginfo("robot spoke: %s", sentence)
		self.pub_speak.publish(sentence)
		self.wait()

	# ログファイルの書き込みの関数
	def log_file(self, sentence):
		if os.path.exists(self.log_file_name) == True:
			with open(self.log_file_name, "a") as f:
				f.write(str(datetime.datetime.now()) + "\t" + "robot heard:" + sentence + "\n")
		else:
			with open(self.log_file_name, "w") as f:
				f.write(str(datetime.datetime.now()) + "\t" + "robot heard:" + sentence + "\n")

	def log_file_spoke(self, sentence):
		if os.path.exists(self.log_file_name) == True:
			with open(self.log_file_name, "a") as f:
				f.write(str(datetime.datetime.now()) + "\t" + "robot spoke:" + sentence + "\n")
		else:
			with open(self.log_file_name, "w") as f:
				f.write(str(datetime.datetime.now()) + "\t" + "robot spoke:" + sentence + "\n")

	# 認識結果の文字列の処理
	def callback(self, data):
		request = data.data
		# 認識した文字列が何か決定する
		request_list = ['follow me', 'stop following me', 'here is the car', 'yes', 'no']
		max = 0
		answer = ''
		for q in request_list:
			level = self.get_similar(request, q)
			if level > max:
				max = level
				answer = q
		self.log_file(answer)
		rospy.loginfo("robot heard: %s", answer)
		self.judge(answer)

	# 条件分岐の処理
	def judge(self, answer):
		# Follow me
		if self.follow_me_flag != 'Finish':
			if answer == 'follow me':
				self.follow_me_flag = 'True'
				self.speak('Should I start following you?')
				self.pub_start.publish(True)
			# 'Yes'の時
			elif (answer == 'yes') and (self.follow_me_flag == 'True'):
				self.follow_me_flag = 'Finish'
				self.stop_flag = 'False'
				self.speak('When we arrive at the destination, please say stop following me.')
				self.pub.publish('start') # 制御に'start'をpublish (***follow me が始まる***)
				print('== Follow me....... ==')
				self.speak('I am ready to start following you.')
				self.pub_start.publish(True)
			# 'No'の時
			elif (answer == 'no') and (self.follow_me_flag == 'True'):
				self.follow_me_flag = 'False'
				self.speak('OK. Please say the command again.')
				self.pub_start.publish(True)
			
			else:
				self.speak('Sorry. I could not understand the command. Please say the command again.')
				self.pub_start.publish(True)

		# Stop follow me
		elif (self.follow_me_flag == 'Finish') and (self.stop_flag != 'Finish'):
			if (answer == 'stop following me') or (answer == 'here is the car'):
				self.stop_flag = 'True'
				self.speak('Should I stop following you?')
				self.pub_start.publish(True)
			# 'Yes'の時
			elif (answer == 'yes') and (self.stop_flag == 'True'):
				self.stop_flag = 'Finish'
				self.follow_me_flag = 'False'
				self.pub.publish('stop') # 制御に'stop'をpublish (***follow me が終わる***)
				self.speak('OK, I will stop. Please give me next command.')
				print('== Stop follow me...... ==')
				next = Activate
				next.id = 1
				self.pub_nlp_first.publish(next)
				self.pub_stop_recognition.publish('stop node')
				os.system('rosnode kill hmc_follow_me_nlp_recognition')
				os.system('rosnode kill hmc_follow_me_nlp_speak')
				os.system('rosnode kill hmc_follow_me_nlp_main')
			# 'No'の時の処理
			elif (answer == 'no') and (self.stop_flag == 'True'):
				self.stop_flag = 'False'
				self.speak('OK. Please say the command again.')
				self.pub_start.publish(True)
			else:
				self.speak('Sorry. I could not understand the command. Please say the command again.')
				self.pub_start.publish(True)

	# 発話が終了したメッセージを受け取る
	def control(self, data):
		self.speak_flag = data.data

	# 発話が終了するまで待つ
	def wait(self):
		self.speak_flag = False
		while self.speak_flag != True:
			pass

	def __init__(self):
		rospy.init_node('hmc_follow_me_nlp_main', anonymous=True)
		rospy.Subscriber('hmc_follow_me_nlp/recognition_result', String, self.callback)
		rospy.Subscriber("hmc_follow_me_nlp/finish_speaking", Bool, self.control) # 発話終了の合図
		self.pub = rospy.Publisher('/follow_me/control', String, queue_size=10) # **制御にFollow me 開始停止の合図**
		self.pub_speak = rospy.Publisher('hmc_follow_me_nlp/speak_sentence', String, queue_size=10)
		self.pub_start = rospy.Publisher('hmc_follow_me_nlp/recognition_start', Bool, queue_size=10)
		self.pub_nlp_first = rospy.Publisher('/help_me_carry/activate', Activate, queue_size=10)
		self.pub_stop_recognition = rospy.Publisher("hmc_follow_me_nlp/stop_recognition", String, queue_size=10) # 音声認識のループを抜ける
		self.follow_me_flag = 'False'
		self.stop_flag = 'False'
		self.log_file_flag = False # ログファイルを書き込みか追加か判定する
		self.speak_flag = False
		self.log_file_name = "{}/log{}.txt".format(os.path.join(os.path.dirname(os.path.abspath(__file__)), "log"), datetime.datetime.now())


if __name__ == '__main__':
	Follow_me_nlp()
	rospy.spin()