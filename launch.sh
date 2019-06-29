DIR=$(cd $(dirname $0); pwd)
echo "$DIR/mapdata/biolink"
pwd
rosrun map_server map_server biolink &
roslaunch hmc_start_node help_me_carry.launch
