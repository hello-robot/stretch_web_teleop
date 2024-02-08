#!/bin/bash

export NODE_EXTRA_CA_CERTS="/home/hello-robot/ament_ws/src/stretch_web_teleop/certificates/rootCA.pem"
cd /home/hello-robot/ament_ws/src/stretch_web_teleop && pm2 start npm --name="stretch_web_teleop" -- run localstorage 
cd /home/hello-robot/ament_ws/src/stretch_web_teleop && pm2 start server.js --watch
cd /home/hello-robot/ament_ws/src/stretch_web_teleop && pm2 start start_robot_browser.js --watch 
set +x
