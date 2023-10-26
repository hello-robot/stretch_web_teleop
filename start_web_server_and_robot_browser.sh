#!/bin/bash

export NODE_EXTRA_CA_CERTS="/home/hello-robot/web_interface_ws/src/stretch_teleop_interface/certificates/rootCA.pem"
cd /home/hello-robot/web_interface_ws/src/stretch_teleop_interface && pm2 start npm --name="stretch_teleop_interface" -- run localstorage 
cd /home/hello-robot/web_interface_ws/src/stretch_teleop_interface && pm2 start server.js --watch
cd /home/hello-robot/web_interface_ws/src/stretch_teleop_interface && pm2 start start_robot_browser.js --watch 
set +x