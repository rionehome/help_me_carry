# hmc_navigation
## Overview
目的地の文字列を受け取り、制御に目的地の文字列を送る。
その後、制御が目的地に着いたらメッセージを受け取り、人検出にメッセージを送る。

## Usage
```
roslaunch hmc_navigation hmc_navigation.launch
```

##
navigationからのメッセージ受け取り
`/navigation/goal`(std_msgs/Bool)