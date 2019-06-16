#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
import get_word
import os
import time
from pocketsphinx import LiveSpeech, get_model_path
loop_count=0
model_path = get_model_path()
dic_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary')



def help():
    def start_speech(data):
        if(data.data=='next'):
            global start_flag
            start_flag=True
            main()
            return
    
   
    def get_yesno(sentence):
        global take_ans
        if(sentence!=''):
            take_ans=sentence.data
            print(take_ans)
            return
        print('I\'m taking yes or no...')
        time.sleep(1)
        yes_no.publish(True)


    def get_txt(sentence):
        global txt
        if(sentence!=''):
            txt=sentence.data
            return
        time.sleep(1)
        start_resume.publish(True)

        
    def start_speaking(sentence):
        global finish_speaking_flag
        finish_speaking_flag=False
        if(sentence!=''):
            print(sentence)
            speak.publish(sentence)
            return

    def finish_speaking(data):
        global finish_speaking_flag
        if(data.data==True):
            finish_speaking_flag=True
            return
        
        
    def main():
        while(1):
            if(start_flag!=False):
                global take_ans, start_flag, loop_count, txt, finish_speaking_flag
                txt=''
                get_txt('')
                while(txt==''):#txt取得まで待機
                    continue
                take_ans=''
                place_list=['bed', 'kitchen', 'car', 'living room']
                word_list=[]
                word_list=get_word.main(txt.decode('utf-8'))
                
                print('place:{}'.format(word_list[0]))
                #os.system("espeak 'I will take this bag to {} OK?'".format(word_list[0]))
                start_speaking('I will take this bag to {} OK?'.format(word_list[0]))
                while(finish_speaking_flag!=True):
                    continue
                
                get_yesno('')#聴きとった内容が正しいかを確認
                while(take_ans!='yes' and take_ans!='no'):#yesかnoを聞き取るまで待機
                    continue
                
                yes_no.publish(False)
                if(take_ans=='yes'):
                    #os.system("espeak 'OK, I take this bag to {}'".format(word_list[0]))
                    start_speaking('OK, I take this bag to {}'.format(word_list[0]))
                    while(finish_speaking_flag!=True):
                        continue
                    start_speaking('Sorry, I have no arm. So, I want you to put your bag on plate.')
                    while(finish_speaking_flag!=True):
                        continue
                    
                    send_place.publish(word_list[0])
                    start_flag=False
                    txt=''
                    break
                #制御へ場所情報を送信.
                else:
                    if(loop_count>=2):
                        #場所情報をランダムに発話していく.
                        for i in place_list:
                            #os.system("espeak 'Is it {}'".format(i))
                            finish_speaking('Is it {} ?'.format(i))
                            while(finish_speaking_flag!=True):
                                continue
                            
                            take_ans=''
                            get_yesno('')
                            while(take_ans!='yes' and take_ans!='no'):
                                continue
                            
                            if(take_ans=='yes'):
                                #os.system("espeak 'OK, I take this bag to {}'".format(i))
                                start_speaking('OK, I take this bag to {}'.format(word_list[0]))
                                while(finish_speaking_flag!=True):
                                    continue
                                start_speaking('Sorry, I have no arm. So, I want you to put your bag on plate.')
                                while(finish_speaking_flag!=True):
                                    continue
                                send_place.publish(i)
                                start_flag=False
                                break
                        txt=''
                        break

                    #os.system("espeak 'Sorry, please say again  where you want to go'")
                    start_speaking('Sorry, please say again where you want to carry')
                    while(finish_speaking_flag!=True):
                        continue
                    
                    loop_count+=1
                    time.sleep(3)
                    txt=''
                    #start_resume.publish('')
                
            
            
    rospy.init_node('help', anonymous=True)
    start_resume=rospy.Publisher('help_me_nlp_second_half/recognition_start', Bool, queue_size=10)#音声認識開始
    yes_no=rospy.Publisher('yes_no_start', Bool, queue_size=10)#yes_no取得開始
    send_place=rospy.Publisher('help_me_carry/send_place', String, queue_size=10)#場所情報送信
    speak=rospy.Publisher('help_me_nlp_second_half/speak_sentence', String, queue_size=10)#発話開始
    rospy.Subscriber('help_ctrl', String, start_speech)#起動用
    rospy.Subscriber('help_me_nlp_second_half/recognition_result', String, get_yesno)#yes_no
    rospy.Subscriber('help_me_nlp_second_half/recognition_result', String, get_txt)#音声認識結果取得
    rospy.Subscriber('help_me_nlp_second_half/finish_speaking', Bool, finish_speaking)
    rospy.spin()




if __name__=='__main__':
    try:
        help()
    except rospy.ROSInterruptException:
        pass
