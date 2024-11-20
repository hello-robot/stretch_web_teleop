#!/bin/bash

# Usage: ./launch_interface.sh -m $HELLO_FLEET_PATH/maps/<map_name>.yaml
MAP_ARG=""
if getopts ":m:" opt && [[ $opt == "m" && -f $OPTARG ]]; then
    echo "Setting map..."
    MAP_ARG="map_yaml:=$OPTARG"
fi

# Usage: ./launch_interface.sh -t pyttsx3
TTS_ARG=""
if getopts ":t:" opt && [[ $opt == "t" ]]; then
    echo "Setting tts engine..."
    TTS_ARG="tts_engine:=$OPTARG"
fi

REDIRECT_LOGFILE="$REDIRECT_LOGDIR/launch_interface.`date '+%Y%m%d%H%M'`_redirected.txt"
cd $HOME/ament_ws/src/stretch_web_teleop
echo "Setup environment..."
. /etc/hello-robot/hello-robot.conf
export HELLO_FLEET_ID HELLO_FLEET_ID
export HELLO_FLEET_PATH=$HOME/stretch_user
source /opt/ros/humble/setup.bash 
source ~/ament_ws/install/setup.bash 
source /usr/share/colcon_cd/function/colcon_cd.sh 

echo "Freeing robot process..."
/usr/bin/python3 $HOME/.local/bin/stretch_free_robot_process.py 

echo "Stopping previous instances..."
./stop_interface.sh 

echo "Reload USB bus..."
sudo udevadm control --reload-rules && sudo udevadm trigger 

echo "Start ROS2..."
sleep 2;
screen -dm -S "web_teleop_ros" ros2 launch stretch_web_teleop web_interface.launch.py $MAP_ARG $TTS_ARG 
sleep 3;
./start_web_server_and_robot_browser.sh