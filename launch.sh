if [[ -z $1 ]]; then
	echo "ロードするデータ名が未定義です。"
	exit 1
fi
DIR=$(cd $(dirname $0); pwd)
echo "$DIR/mapdata/$1"
gnome-terminal -x  bash -c "
	rosrun map_server map_server $1
" &
roslaunch hmc_start_node help_me_carry.launch
