#!/bin/bash
set -o pipefail

while getopts m:t:f opt; do
    case $opt in
        m)
            # Usage: ./launch_interface.sh -m $HELLO_FLEET_PATH/maps/<map_name>.yaml
            if [[ -f $OPTARG ]]; then
                MAP="-m $OPTARG"
            fi
            ;;
        t)
            # Usage: ./launch_interface.sh -t pyttsx3
            TTS="-t $OPTARG"
            ;;
        f)
            # Usage: ./launch_interface.sh -f
            FIREBASE="-f"
    esac
done

timestamp='stretch_web_teleop_'`date '+%Y%m%d%H%M'`;
logdir="$HOME/stretch_user/log/web_teleop/$timestamp"
logfile_ros="$logdir/start_ros2.txt"
logfile_node="$logdir/start_web_server_and_robot_browser.txt"
logzip="$logdir/stretch_web_teleop_logs.zip"
mkdir -p $logdir

function echo_failure_help {
    zip -r $logzip $logdir/ > /dev/null
    echo ""
    echo "#############################################"
    echo "FAILURE. COULD NOT LAUNCH WEB TELEOP."
    echo "Look at the troubleshooting guide for solutions to common issues: https://docs.hello-robot.com/0.3/getting_started/demos_web_teleop/#troubleshooting"
    echo "or contact Hello Robot support and include $logzip"
    echo "#############################################"
    echo ""
    exit 1
}

echo "#############################################"
echo "LAUNCHING WEB TELEOP"
echo "#############################################"

cd $HOME/ament_ws/src/stretch_web_teleop
./start_ros2.sh -l $logdir $MAP $TTS |& tee $logfile_ros
if [ $? -ne 0 ]; then
    echo_failure_help
fi

# echo ""
cd $HOME/ament_ws/src/stretch_web_teleop
./start_web_server_and_robot_browser.sh -l $logdir $FIREBASE |& tee $logfile_node
if [ $? -ne 0 ]; then
    echo_failure_help
fi

zip -r $logzip $logdir/ > /dev/null

echo ""
echo "#############################################"
echo "DONE! WEB TELEOP IS UP!"
echo "Visit the URL(s) below to see the web interface:"
if [ "$FIREBASE" = "-f" ]; then
    echo "https://web.hello-robot.com/"
else
    echo "https://localhost/operator"
    ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/https:\/\/\2\/operator/p'
fi
echo "#############################################"
echo ""
