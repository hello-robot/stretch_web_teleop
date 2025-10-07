#!/bin/bash
set -e

REDIRECT_LOGDIR="$HOME/stretch_user/log/web_teleop"
mkdir -p $REDIRECT_LOGDIR
while getopts l:m:t: opt; do
    case $opt in
        l)
            # Usage: ./start_ros2.sh -l /tmp/some_folder
            if [[ -d $OPTARG ]]; then
                REDIRECT_LOGDIR=$OPTARG
            fi
            ;;
        m)
            # Usage: ./start_ros2.sh -m $HELLO_FLEET_PATH/maps/<map_name>.yaml
            if [[ -f $OPTARG ]]; then
                echo "Setting map..."
                MAP_ARG="map_yaml:=$OPTARG"
            fi
            ;;
        t)
            # Usage: ./start_ros2.sh -t pyttsx3
            echo "Setting tts engine..."
            TTS_ARG="tts_engine:=$OPTARG"
            ;;
    esac
done
REDIRECT_LOGFILE="$REDIRECT_LOGDIR/start_ros2.`date '+%Y%m%d%H%M'`_redirected.txt"
echo "Arguments:" &>> $REDIRECT_LOGFILE
echo "-l $REDIRECT_LOGDIR" &>> $REDIRECT_LOGFILE
echo "-m $MAP_ARG" &>> $REDIRECT_LOGFILE
echo "-t $TTS_ARG" &>> $REDIRECT_LOGFILE

# Disabled on Sept 9th bc it is triggering when the wifi adapter is something else, like wlp0s20f3
#if [[ -z `nmcli -t -f DEVICE c show --active | grep wlo1` ]]; then
#    if ! command -v wifi-connect 2>&1 >/dev/null
#    then
#        echo "Not connected to Wifi, but Wifi-Connect CLI is missing. Doing nothing..."
#    else
#        echo "Not connected to Wifi. Starting Wifi-Connect..."
#        echo "Please connect to $HOSTNAME wifi and provide your home network's credentials"
#        sudo wifi-connect -s $HOSTNAME &>> $REDIRECT_LOGFILE
#    fi
#fi

echo "Setup environment..."
. /etc/hello-robot/hello-robot.conf
export HELLO_FLEET_ID HELLO_FLEET_ID
export HELLO_FLEET_PATH=$HOME/stretch_user
source /opt/ros/humble/setup.bash &>> $REDIRECT_LOGFILE
source ~/ament_ws/install/setup.bash &>> $REDIRECT_LOGFILE
source /usr/share/colcon_cd/function/colcon_cd.sh &>> $REDIRECT_LOGFILE

echo "Freeing robot process..."
/usr/bin/python3 $HOME/.local/bin/stretch_free_robot_process.py &>> $REDIRECT_LOGFILE

echo "Stopping previous instances..."
./stop_interface.sh &>> $REDIRECT_LOGFILE

echo "Reload USB bus..."
sudo udevadm control --reload-rules && sudo udevadm trigger &>> $REDIRECT_LOGFILE

echo "Start ROS2..."
sleep 2;
screen -dm -S "web_teleop_ros" ros2 launch stretch_web_teleop web_interface.launch.py $MAP_ARG $TTS_ARG &>> $REDIRECT_LOGFILE
sleep 3;
