# follow_me_nlp
## Overview
Follow meの自然言語処理部分のパッケージです。

## Setup
以下のコマンドで必要なパッケージをインストールする必要あり  
【PocketSphinxのインストール】  
`git clone https://github.com/cmusphinx/pocketsphinx-python.git`  
`sudo apt-get install -y python python-dev python-pip build-essential swig git libpulse-dev`  
`sudo pip install pocketsphinx`  
`sudo apt-get install -y libttspico-utils`   

【Julius(英語版)のダウンロード】  
https://sourceforge.net/projects/juliusmodels/files/  
このサイトのENVR-v5.4.Dnn.Bin.zipをダウンロードして展開する  

## Description
* main.py:　自然言語処理のコードです。

	処理できる単語は以下です。

	* 「Follow me」
	* 「Stop following me」か「Here is the car」
	* 「Yes」
	* 「No」

* recognition.py：　**PocketSphinx**での音声認識のテスト用のコードです。

* recognition_english_julius.py：　**Julius**(英語版)での音声認識のテスト用のコードです。

* speak.py：　発話のテスト用のコードです。

* `/follow_me_nlp/src/log/`：　音声認識結果の文と発話の文のログが書き込まれます。

* `/follow_me_nlp/src/dictionary/`:　単語辞書と文法辞書があります。

## Usage
* PockeSphinx

```
roslaunch follow_me_nlp follow_me_nlp.launch
```



* Julius(英語版)


`/follow_me_nlp/src/dictionary/follow_me_julius.dict`を単語辞書に使ってモジュールモードでJuliusを起動する。  

```
roslaunch follow_me_nlp follow_me_nlp_julius_english.launch
````


## Node
**`name` follow_me_nlp_main**

### Subscribe Topic

* **`follow_me_nlp/recognition_result`** 音声認識結果の受け取り （ std_msgs/String ）

* **`follow_me_nlp/finish_speaking`** 発話が終了した合図の受け取り ( std_msgs/Bool )

### Publish Topic

* **`follow_me_nlp/follow_me`** 制御にFollow meの開始と終了の合図の文字列を渡す ( std_msgs/String )

	'start'：「Follow me」開始の合図

	'stop'：「Follow me」終了の合図

* **`follow_me_nlp/speak_sentence`** 発話する文字列 ( std_msgs/String )

* **`follow_me_nlp/recognition_start`** 音声認識再開 ( std_msgs/Bool )

	True：音声認識　開始  
	False:音声認識　停止

## Node
**`name` follow_me_nlp_recognition**

**`name` follow_me_nlp_recognition_julius_english**

### Subscribe Topic

* **`follow_me_nlp/recognition_start`** 音声認識再開の受け取り （ std_msgs/Bool ）

	True：音声認識　開始  
	False:音声認識　停止

### Publish Topic

* **`follow_me_nlp/recognition_result`** 音声認識結果 ( std_msgs/String )

## Node
**`name` follow_me_nlp_speak**

### Subscribe Topic

* **`follow_me_nlp/speak_sentence`** 発話する文字列の受け取り （ std_msgs/String ）

### Publish Topic

* **`follow_me_nlp/finish_speaking`** 発話が終了した合図 ( std_msgs/Bool )
