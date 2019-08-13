# Help Me Carry

コミットはdevelopブランチで！！！

## Launch
turtlebot等,各種センサと接続時に以下のファイルを実行.  
`roslaunch help_me_carry help_me_carry.launch`

センサなどとの接続なし、テスト用には以下のファイルを実行.  
`rosrun help_me_carry test_hmc.launch`

## Subscriber Topic
- `/natural_language_processing/start` (std_msgs/String)  
全体の開始Subscriber,  空文字をsubscribeすることで起動.  
競技を始める場合は,ここに空文字をpublishする.

- `/natural_language_processing/follow_me` (std_msgs/String)  
follow me開始Subscriber, 空文字をsubscribeすることで起動.  
follow meの音声認識から開始したい場合は,ここに空文字をpublishする.  

- `/natural_language_processing/stop_follow_me` (std_msgs/String)  
follow me終了Subscriber, 空文字をsubscribeすることで起動.  
車の位置情報をlocationに送信し,"take this bag..."の音声認識から開始したい場合は,
ここに空文字をpublishする.

- `/natural_language_processing/ask_put_bag`　(std_msgs/String)  
バッグを置いたかを尋ねる, 空文字をsubscribeすることで起動.  
"take this bag..."の認識後にバッグを置くように促す.  
ここからの起動は,場所情報が獲得できないため不可.

- `/natural_language_processing/restart_ask_put_bag` (std_msgs/String)
再度バッグを置いたかを尋ねる, 空文字をsubscribeすることで起動.  
一度,バッグを置いたかを確認した際に否定された場合,時間を置いて再度確認する.  
上記と同様の理由から,ここからの起動は不可.

- `/natural_language_processing/ask_all_places` (std_msgs/String)  
場所情報を相手に全て確認する, 空文字をsubscribeすることで起動.  
"take this bag..."の音声認識結果を相手に確認した際に,3回否定された場合に全ての場所を相手に聞く.  
まず,呼び出されることはない.

- `/natural_language_processing/release_bag` (std_msgs/String)
kitchen等到着後バッグをアームから放す, 空文字をsubscribeすることで起動  
人間検出から開始したい場合は, ここに空文字をpublishする.

- `/natural_language_processing/back_to_the_car` (std_msgs/String)
人間検出後,車に戻るためのSubscriber, 空文字をsubscribeすることで起動.  
人間検出後,車に戻るところから開始したい場合はここに空文字をpublishする.


## Publisher Topic
- `/follow_me/control` (std_msgs/String)  
follow meの開始・終了用Publisher, 開始時:"start", 終了時:"stop"を送信  

- `/hmc/send_place_msg` (std_msgs/String)  
"take this bag..."で認識したバッグの送り先情報が保持できないため,
バッグの送り先情報をpublishするノードであるsend_place_infoに予めpublishする.

- `/human_detection/start` (std_msgs/String)  
human detectionの開始用Publisher, 開始時:"start"