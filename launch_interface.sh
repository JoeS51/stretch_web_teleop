#!/bin/bash

# Usage: ./launch_interface -m $HELLO_FLEET_PATH/maps/<map_name>.yaml
MAP_ARG=""
if getopts ":m:" opt && [[ $opt == "m" && -f $OPTARG ]]; then
    echo "Setting map..."
    MAP_ARG="map_yaml:=$OPTARG"
fi

stretch_free_robot_process.py;
sudo udevadm control --reload-rules && sudo udevadm trigger
source /opt/ros/humble/setup.bash
source ~/joe_brian_ws/install/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
sleep 2;
screen -dm -S "web_teleop_ros" ros2 launch stretch_web_teleop web_interface.launch.py $MAP_ARG
sleep 3;
~/joe_brian_ws/src/stretch_web_teleop/start_web_server_and_robot_browser.sh
