#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 音声認識

import rospy
from std_msgs.msg import String, Bool
import os
import socket
import sys
import time

class Recognition:
	def recognition(self):
		recogout = False
		while 1:
			response = self.client.recv(1024)
			text = ""
			if ("<RECOGOUT>" in response) and ("</RECOGOUT>" in response):
				data = response
				text = data
				data = ""
			elif "<RECOGOUT>" in response:
				data = response
				recogout = True
			elif ("</RECOGOUT>") in response:
				data = data + response
				text = data
				data = ""
				recogout = False
			elif recogout == True:
				data = data + response

			if ("<RECOGOUT>" in text) and ("</RECOGOUT>" in text):
				self.client.send("PAUSE\n")
				print('== STOP RECOGNITION ==')
				text_list = text.split("    ")
				data2 = ""
				for j in text_list:
					if "<WHYPO WORD=" in j:
						word = j.split("\"")[1]
						if data2 == "":
							data2 = word
						else:
							data2 = data2 + " " + word
				data2 = data2.replace("<s>", "")
				data2 = data2.replace("</s>", "")
				data2 = data2.strip()
				self.pub.publish(data2)

	# 音声認識再開のメッセージを受け取る
	def control(self, data):
		if data.data == True:
			self.client.send("RESUME\n")
			print('== START RECOGNITION ==')
		if data.data == False:
			self.cliant("PAUSE\n")
			print('== STOP RECOGNITION ==')

	def __init__(self):
		rospy.init_node('follow_me_nlp_recognition_julius_english', anonymous=True)
		rospy.Subscriber('follow_me_nlp/recognition_start', Bool, self.control) # 音声認識開始の合図
		self.pub = rospy.Publisher('follow_me_nlp/recognition_result', String, queue_size=10) # 音声認識結果

		HOST = "localhost"
		PORT = 10500
		self.client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		self.client.connect((HOST,PORT))
		#self.client.send("PAUSE\n") # ノードを立ち上げた時から音声認識が始めないときに使う
		#print('== STOP RECOGNITION ==')
		self.recognition()

if __name__ == '__main__':
	Recognition()
	rospy.spin()