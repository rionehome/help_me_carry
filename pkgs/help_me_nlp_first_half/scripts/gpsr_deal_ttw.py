#!/usr/bin/env python
# -*- coding: utf-8 -*-
import treetaggerwrapper as ttw
import numpy as np
import os
import glob
import rospy
from pocketsphinx import LiveSpeech, get_model_path
from std_msgs.msg import String
from gpsr.msg import Ans
import os
import sys


speak=['answer', 'tell', 'say', 'speak', 'talk', 'explain', 'teach', 'express', 'mouth', 'mention', 'utter']
go=['go', 'come', 'move', 'run', 'transfer', 'shift', 'travel', 'follow']
carry=['carry', 'accompany', 'escort', 'deliver', 'bring', 'place', 'navigate', 'locate', 'send', 'bear', 'channel', 'transport', 'transmit', 'submit']
find=['find', 'look', 'meet', 'get', 'pick', 'search', 'seek', 'hunt', 'watch']

TTWDIR='/home/takoyan/treetag/'
tagger=ttw.TreeTagger(TAGLANG='en', TAGDIR=TTWDIR)



def gpsr_deal():
    """
    get_Index_of_nextNoun関数
    入力された文章内における,指定された単語の次に来る名詞のindex番号を返す.
    """
    def get_Index_of_nextNoun(sentence, word):
        index_of_word=sentence.index(word)
        for k in range(index_of_word+1, len(sentence)):
            if (result[k][1].startswith('N')):
                return k
        for k in range(index_of_word+1, len(sentence)):
            if  (result[k][1].startswith('PP')):
                return k






    """
    select_Verb関数
    入力された文章内の動詞を分類する.
    返り値として動詞の単語が返還される.
    """
    def select_Verb(sentence):
        for verbs in (speak, go, carry, find):
            for verb in verbs:
                if verb in sentence[0]:
                    return verb   #動詞を返す



    def look_func(sentence, verb):
        ans=[]
        if 'from' in sentence: #fromが文中にある場合
            index_of_from=get_Index_of_nextNoun(sentence, 'from')
            index_of_verb=get_Index_of_nextNoun(sentence, verb)
            place=result[index_of_from][0] #場所を示す名詞を格納
            destination=result[index_of_verb][0] #目的地を示す名詞を格納
            index_of_dest=get_Index_of_nextNoun(sentence, destination)
            thing=result[index_of_dest][0]
            ans.append(place)
            ans.append(thing)
            ans.append(destination)
            return ans

        if 'for' in sentence:
            index_of_for=get_Index_of_nextNoun(sentence, 'for')
            thing=result[index_of_for][0]
            index_of_verb=get_Index_of_nextNoun(sentence, thing)
            place=result[index_of_verb][0]
            ans.append(place)
            ans.append(thing)
            return ans

        place=result[get_Index_of_nextNoun(sentence, verb)][0]
        ans.append(place)
        return ans



    def carry_func(sentence, verb):
        ans=[]
        if 'from' in sentence:
            return look_func(sentence, verb)


        if 'to' in sentence:
            index_of_to=get_Index_of_nextNoun(sentence, 'to')
            index_of_verb=get_Index_of_nextNoun(sentence, verb)
            place=result[index_of_to][0]
            thing=result[index_of_verb][0]
            ans.append(place)
            ans.append(thing)
            return ans
        else:
            index_of_verb=get_Index_of_nextNoun(sentence, verb)
            place=result[index_of_verb][0]
            ans.append(place)

            index_of_next=get_Index_of_nextNoun(sentence, place)
            if index_of_next!=True:
                return ans
            thing=result[index_of_next][0]
            ans.append(thing)
            return ans




    def go_func(sentence, verb):
        ans=[]
        if 'to' in sentence:
            index_of_to=get_Index_of_nextNoun(sentence, 'to')
            place=result[index_of_to][0]
            ans.append(place)
            return ans

        if 'after' in sentence:
            index_of_after=get_Index_of_nextNoun(sentence, 'after')
            follow=result[index_of_after][0]
            if len(ans)<=0:
                ans=['empty']*4
                ans[3]=follow
            return ans
        else:
            index_of_noun=get_Index_of_nextNoun(sentence, verb)
            follow=result[index_of_noun][0]
            if len(ans)<=0:
                ans=['empty']*4
                ans[3]=follow
            return ans


    def cut_sentence(result):
        verb_count=[]
        txt_list=[]   
        for num, word_class in enumerate(result):
            if word_class[1].startswith('VV'):
                verb_count.append(num)#動詞の格納されているインデックスを獲得

        for num in range(len(verb_count)):
            if num+1==len(verb_count):
                txt_list.append(result[verb_count[num]:])
                break
            txt_list.append(result[verb_count[num]:verb_count[num+1]])

        return txt_list





    def start_speech(data):
        global start_flag
        start_flag=True
        print('ok')
        start_record.publish('')#録音の開始合図
        return 



    def next_order(data):
        global next_flag
        global continue_flag
        next_flag=True
        if(data.data=='all message were sent'):
            continue_flag=True
        return





    def main(sentence):
        #sentence='bring me this apple'
        #sentence='bring me this apple from the kitchen'
        #sentence='bring this apple to him'
        #sentence='look for the fruit in the corridor'
        #sentence='go to the kitchen'
        #sentence='go after him'
        #sentence='go to the sofa, meet Emily, and follow her'
        #sentence='meet Emily'
        #sentence=sentence.lower()
        sentence=(sentence.data).decode('utf-8')#treetagger用にutf-8へ
        print(sentence)

        global result
        global sentence_num
        global next_flag
        global continue_flag
        txt_list=[]#各命令文を分割したもののうち,文章部分のみを全て格納するリスト
        result=[]#各命令分を分割したもののうち,文章と品詞情報を文章単位で格納するリスト

        start_flag=False
        next_flag=False
        continue_flag=True


        tags=tagger.TagText(sentence)
        for i in tags:
            li=i.split()
            result.append(li[:2]) #('動詞', '品詞')

        result_list=cut_sentence(result)#上記のリストresultの親となる全ての文章と品詞情報を格納するリスト.
        print(result_list)
        sentence_num=len(result_list)#命令が何個存在するか



        while(sentence_num>=1):
            answer=[]#最終的にsubscriberへ送るデータを格納するリスト
            sentence=''
            result=result_list.pop(0)
            sentence_num=len(result_list)
            for word in result:
                sentence+=word[0]+' '
            sentence=sentence.split()
            verb=select_Verb(sentence) #文中の動詞を獲得

            if verb in carry:
                answer=carry_func(sentence, verb)
            elif verb in find:
                answer=look_func(sentence, verb) #[place, thing]の順
            elif verb in go:
                answer=go_func(sentence, verb)


            place='empty'
            thing='empty'
            destination='empty'
            follow='empty'

            if len(answer)>=4:
                follow=answer[3].encode()
            if len(answer)>=3:
                destination=answer[2].encode()
            if len(answer)>=2:
                thing=answer[1].encode()
            if len(answer)>=1:
                place=answer[0].encode()



            pub=rospy.Publisher('gpsr_deal', Ans, queue_size=100)
            box=Ans()
            box.place=place
            box.thing=thing
            box.destination=destination
            box.follow=follow
            box.sentence_num=sentence_num
            print('----------infomation----------')
            print(box)
            print('------------------------------')
            rospy.sleep(5)
            pub.publish(box)


            
    rospy.init_node('gpsr_deal', anonymous=True)
    start_record=rospy.Publisher('sound_system/recognition', String, queue_size=10)
    rospy.Subscriber('gpsr_ctrl', String, start_speech)
    rospy.Subscriber('sound_system/recognition/result', String, main)
    rospy.Subscriber('next_order', String, next_order)
    rospy.spin()



        
if __name__=='__main__':
    try:
        gpsr_deal()
    except rospy.ROSInterruptException:
        pass
