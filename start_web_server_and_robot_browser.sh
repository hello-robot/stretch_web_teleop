#!/bin/bash

# Usage: ./start_web_server_and_robot_browser.sh -s localstorage
STORAGE_ARG="localstorage"
if getopts ":s:" opt && [[ $opt == "s" ]]; then
    echo "got $OPTARG"
    if [[ "$OPTARG" == "localstorage" ]]; then
        echo "Setting storage model to localstorage..."
    elif [[ "$OPTARG" == "firebase" ]]; then
        echo "Setting storage model to firebase..."
        STORAGE_ARG="firebase"
    else
        echo "$OPTARG storage model not found! Setting to storage model to localstorage..."
    fi
fi

export NODE_EXTRA_CA_CERTS="/home/hello-robot/ament_ws/src/stretch_web_teleop/certificates/rootCA.pem"
cd ~/ament_ws/src/stretch_web_teleop && pm2 start -s npm --name="stretch_web_teleop" -- run $STORAGE_ARG
cd ~/ament_ws/src/stretch_web_teleop && pm2 start -s server.js --watch
cd ~/ament_ws/src/stretch_web_teleop && pm2 start -s start_robot_browser.js --watch
set +x
echo ""
echo "Visit the URL(s) below to see the web interface:"
echo "https://localhost/operator"
ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/https:\/\/\2\/operator/p'
