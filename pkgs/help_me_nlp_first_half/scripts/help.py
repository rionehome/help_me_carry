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

loop_count = 0
model_path = get_model_path()
dic_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary')
navigation_wait = False
activate = False


def help():
	def start_speech(data):
		if (data.id == 1):
			print "first_nlp"
			global start_flag, activate
			start_flag = True
			activate = True
			main()

	def get_yesno(sentence):
		global take_ans
		if (sentence != ''):
			take_ans = sentence.data
			print(take_ans)
			return
		print('I\'m taking yes or no...')
		time.sleep(1)
		yes_no.publish(True)

	def get_txt(sentence):
		global txt
		if (sentence != ''):
			txt = sentence.data
			return
		time.sleep(1)
		start_resume.publish(True)

	def start_speaking(sentence):
		global finish_speaking_flag
		finish_speaking_flag = False
		if (sentence != ''):
			print(sentence)
			speak.publish(sentence)
			return

	def finish_speaking(data):
		global finish_speaking_flag
		if (data.data == True):
			finish_speaking_flag = True
			return

	def navigation_goal_callback(data):
		global activate
		if activate:
			# 次のノードに処理を渡す
			next = Activate()
			next.id = 2
			next_pub.publish(next)
			activate = False

	def send_place_msg(place):
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
			next_pub.publish(next)

	def main():
		while (1):
			if (start_flag != False):
				global take_ans, start_flag, loop_count, txt, finish_speaking_flag
				txt = ''
				get_txt('')
				while (txt == ''):  # txt取得まで待機
					continue
				print(txt)
				take_ans = ''
				place_list = ['bed', 'kitchen', 'car', 'living room']
				word_list = get_word.main(txt.decode('utf-8'))

				print('place:{}'.format(word_list[0]))
				# os.system("espeak 'I will take this bag to {} OK?'".format(word_list[0]))
				start_speaking('I will take this bag to {} OK?'.format(word_list[0]))
				while (finish_speaking_flag != True):
					continue

				get_yesno('')  # 聴きとった内容が正しいかを確認
				while (take_ans != 'yes' and take_ans != 'no'):  # yesかnoを聞き取るまで待機
					continue

				yes_no.publish(False)
				if (take_ans == 'yes'):
					# os.system("espeak 'OK, I take this bag to {}'".format(word_list[0]))
					start_speaking('OK, I take this bag to {}'.format(word_list[0]))
					while (finish_speaking_flag != True):
						continue
					start_speaking('Sorry, I have no arm. So, I want you to put your bag on plate.')
					while (finish_speaking_flag != True):
						continue

					# navigationに場所を伝え、移動終了まで処理をする
					send_place_msg(word_list[0])
					start_flag = False
					txt = ''
				else:
					if (loop_count >= 2):
						# 場所情報をランダムに発話していく.
						for place in place_list:
							# os.system("espeak 'Is it {}'".format(i))
							start_speaking('Is it {} ?'.format(place))
							while (finish_speaking_flag != True):
								continue

							take_ans = ''
							get_yesno('')
							while (take_ans != 'yes' and take_ans != 'no'):
								continue

							if (take_ans == 'yes'):
								# os.system("espeak 'OK, I take this bag to {}'".format(i))
								start_speaking('OK, I take this bag to {}'.format(place))
								while (finish_speaking_flag != True):
									continue
								start_speaking('Sorry, I have no arm. So, I want you to put your bag on plate.')
								while (finish_speaking_flag != True):
									continue
								# navigationに場所を伝え、移動終了まで処理をする
								send_place_msg(place)
								start_flag = False
								break
						txt = ''
						break

					# os.system("espeak 'Sorry, please say again  where you want to go'")
					start_speaking('Sorry, please say again where you want to carry')
					while (finish_speaking_flag != True):
						continue

					loop_count += 1
					time.sleep(3)
					txt = ''

	# start_resume.publish('')

	rospy.init_node('help_me_nlp_first_half_help', anonymous=True)
	start_resume = rospy.Publisher('txt_start', Bool, queue_size=10)  # 音声認識開始
	yes_no = rospy.Publisher('yes_no/recognition_start', Bool, queue_size=10)  # yes_no取得開始
	speak = rospy.Publisher('help_me_nlp_second_half/speak_sentence', String, queue_size=10)  # 発話開始
	next_pub = rospy.Publisher('/help_me_carry/activate', Activate, queue_size=10)

	rospy.Subscriber('/help_me_carry/activate', Activate, start_speech)
	rospy.Subscriber('yes_no/recognition_result', String, get_yesno)  # yes_no
	rospy.Subscriber('recognition_txt', String, get_txt)  # 音声認識結果取得
	rospy.Subscriber('help_me_nlp_second_half/finish_speaking', Bool, finish_speaking)
	rospy.Subscriber('/navigation/goal', Bool, navigation_goal_callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		help()
	except rospy.ROSInterruptException:
		pass
