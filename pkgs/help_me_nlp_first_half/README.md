# help_me_carry
## Overview
  help_me_carryにおける,対話者からのメッセージ処理を行う.


## Description
help.pyは対話者からのメッセージを処理行う。  
resume_sphinx.pyは**PocketSphinx**での音声認識のコード。    

## Usage
```
roslaunch help_me_carry help.launch
```
上記のコマンドを実行し,`help_ctrl`にString型のメッセージを投げると起動する。

## Node
**`name` help**

### Subscribe Topic

* **`/help_me_carry/activate`** メインノードの立ち上げ（ hmc_start_node/Activate ）

* **`/help_me_nlp_first/recognition_txt`** 音声認識結果の受け取り（ std_msgs/String ）

* **`/hmc_follow_me_nlp/finish_speaking`** 発話終了を受け取る( std_msgs/Bool )

* **`/navigation/goal`** 移動終了を受け取る( std_msgs/Bool )

### Publish Topic

* **`/help_me_nlp_first/resume_sphinx`** 音声認識開始 ( std_msgs/String )

* **`/hmc_follow_me_nlp/speak_sentence`** 文章の発話( std_msgs/String )

* **`help_me_carry/send_place`** 場所情報の送信（ std_msgs/String）

* **`/help_me_carry/activate`** 次のノードの立ち上げ（ hmc_start_node/Activate ）


**`name` resume_sphinx.py**

### Subscribe Topic
* **`/help_me_nlp_first/resume_sphinx`** 音声認識の開始を受け取る(std_msgs/String)

### Publish Topic
* **`/help_me_nlp_first/recognition_txt`** 音声認識結果を送信(std_msgs/String)

