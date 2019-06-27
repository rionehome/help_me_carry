# help_me_nlp_second_half
## Overview
Help me carryの後半部分の自然言語処理部分のパッケージです。

## Setup
【PocketSphinx】  
`git clone https://github.com/cmusphinx/pocketsphinx-python.git`

`sudo apt-get install -y python python-dev python-pip build-essential swig git libpulse-dev`

`sudo pip install pocketsphinx`

`sudo apt-get install -y libttspico-utils`

【Julius(英語版)】  
https://sourceforge.net/projects/juliusmodels/files/  
このサイトのENVR-v5.4.Dnn.Bin.zipをダウンロードして展開する  

## Description
* main.py：　自然言語処理のコードです。

* recognition.py：　**PocketSphinx**での音声認識のテスト用のコードです。

* recognition_english_julius.py：　**Julius**(英語版)での音声認識のテスト用のコードです。

* speak.py：　発話のテスト用のコードです。

* `/help_me_nlp_second_half/src/log/`：　音声認識結果の文と発話の文のログが書き込まれます。

* `/help_me_nlp_second_half/src/dictionary/`：　単語辞書と文法辞書があります。

## Usage
- PockeSphinx

```
roslaunch help_me_nlp_second_half help_me_nlp_second_half.launch
```

- Julius(英語版)

`/help_me_nlp_second_half/src/dictionary/yes_no_julius.dict`を単語辞書に使ってモジュールモードでJuliusを起動する。

```
roslaunch help_me_nlp_second_half help_me_nlp_second_half_julius_english.launch
```

* 上のどちらかのコマンドを実行した後、メッセージ名`help_me_nlp_second_half/person_dictation`にBool型のメッセージを投げるとプログラムが動く。
（人認識が終了したら開始する）

## Node
**`name` help_me_nlp_second_half_main**

### Subscribe Topic
* **`/help_me_carry/activate`** 画像認識終了の合図の受け取り （ hmc_start_node/Activate ）

* **`/hmc_follow_me_nlp/finish_speaking`** 発話終了の合図の受け取り ( std_msgs/Bool )

* **`/help_me_nlp_second_half/recognition_result`** 音声認識結果の受け取り（ std_msgs/String ）

* **`/navigation/goal`** 制御の車に着いた合図の受け取り( std_msgs/Bool ）

### Publish Topic
* **`/hmc_follow_me_nlp/speak_sentence`** 発話する文字列 ( std_msgs/String )

* **`/help_me_nlp_second_half/recognition_start`** 音声認識再開 ( std_msgs/Bool )

	True：音声認識　開始  
	False:音声認識　停止

* **`/help_me_carry/activate`** 次のノード起動 （ hmc_start_node/Activate ）

## Node
**`name` help_me_nlp_second_half_recognition**

**`name` help_me_nlp_second_half_julius_english**

### Subscribe Topic

* **`/help_me_nlp_second_half/recognition_start`** 音声認識再開の受け取り （ std_msgs/Bool ）

	True：音声認識　開始  
	False:音声認識　停止

* **`help_me_nlp_second_half/stop_recognition`** 音声認識のループを抜ける ( std_msgs/String ）(help_me_nlp_second_half_recognitionのノードのみ)
	"stop node"

### Publish Topic

* **`/help_me_nlp_second_half/recognition_result`** 音声認識結果 ( std_msgs/String )
