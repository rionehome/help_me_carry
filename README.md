# Help Me Carry

## ブランチ(nenecchi_dev)

ファイル名の接頭語の通り

1. first_follow.py  ( follow_me から Here is the car まで )
2. second_carry.py  ( take this bag to the ~ から バッグを置くところまで )
3. third_human_detect.py    ( 人間検出(ただし現状は仕様不明のため、まだ中身はほぼ空っぽ) )
4. fourth_go_back.py    ( 近くのオペレータと話すところから、車に戻って終了するまで )

これらの順に処理が流れる

## 仕様

すべてのクラスは AbstractModule(abstract_module.py) クラスを継承している。

・ROS の init_node

・Activate関連の共通処理

・どのファイルでも共通するような細々とした関数群

これら3つの機能を持っている

基本的にはこのAbstractModuleクラスの init()内の処理が行われ、次に継承したクラスのinit()処理が行われる

詳しくは AbstractModuleクラスを参照

## Launch
全体の実行はこれ  
`roslaunch help_me_carry help_me_carry.launch`

ID = 1 を送ることで first_follow.py の中の FirstFollow が実行される
`rostopic pub /help_me_carry/activate hmc_start_node/Activate "id: 1 text: ''"`
