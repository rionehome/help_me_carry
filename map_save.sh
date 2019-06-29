if [[ -z $1 ]]; then
	echo "保存するデータ名が未定義です。"
	exit 1
fi
cd mapdata
mkdir $1
rosrun map_server map_sarver -f $1
