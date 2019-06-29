#!/usr/bin/env python
# -*- coding: utf-8 -*-
import treetaggerwrapper as ttw
import os


def get_Index_of_nextNoun(sentence, word, result):
    index_of_word = sentence.index(word)
    for k in range(index_of_word + 1, len(sentence)):
        if (result[k][1].startswith('N')):
            return k
    for k in range(index_of_word + 1, len(sentence)):
        if (result[k][1].startswith('PP')):
            return k


def select_Verb(sentence, word_list):
    for verb in word_list:
        if verb in sentence:
            return verb  # 動詞を返す


def carry_func(sentence, verb, result):
    ans = []
    if 'to' in sentence:
        index_of_to = get_Index_of_nextNoun(sentence, 'to', result)
        index_of_verb = get_Index_of_nextNoun(sentence, verb, result)
        place = result[index_of_to][0]
        thing = result[index_of_verb][0]
        ans.append(place)
        ans.append(thing)
        return ans
    else:
        index_of_verb = get_Index_of_nextNoun(sentence, verb, result)
        place = result[index_of_verb][0]
        ans.append(place)
        
        index_of_next = get_Index_of_nextNoun(sentence, place, result)
        if index_of_next != True:
            return ans
        thing = result[index_of_next][0]
        ans.append(thing)
        return ans


def cut_sentence(result):
    verb_count = []
    txt_list = []
    for num, word_class in enumerate(result):
        if word_class[1].startswith('VV'):
            verb_count.append(num)  # 動詞の格納されているインデックスを獲得
    
    for num in range(len(verb_count)):
        if num + 1 == len(verb_count):
            txt_list.append(result[verb_count[num]:])
            break
        txt_list.append(result[verb_count[num]:verb_count[num + 1]])
    
    return txt_list


def main(txt):
    TTWDIR = os.environ['HOME'] + '/tree-tagger-install/'
    tagger = ttw.TreeTagger(TAGLANG='en', TAGDIR=TTWDIR)
    
    carry = ['carry', 'accompany', 'escort', 'deliver', 'bring', 'place', 'navigate', 'locate', 'send', 'bear',
             'channel', 'transport', 'transmit', 'submit', 'take']
    
    sentence = (txt).decode('utf-8')  # treetagger用にutf-8へ
    
    result = []
    answer = []
    tags = tagger.TagText(sentence)
    
    for i in tags:
        li = i.split()
        result.append(li[:2])  # ('動詞', '品詞')
    
    result_list = cut_sentence(result)
    
    sentence = ''
    result = result_list.pop(0)
    
    for word in result:
        sentence += word[0] + ' '
    sentence = sentence.split()
    verb = select_Verb(sentence, carry)  # 文中の動詞を獲得
    if verb in carry:
        answer = carry_func(sentence, verb, result)
    return answer
