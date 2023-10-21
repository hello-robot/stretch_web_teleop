#!/bin/bash

# PORT_NUMBER=3000
# lsof -i tcp:${PORT_NUMBER} | awk 'NR!=1 {print $2}' | xargs kill 
# kill $(lsof -t -i:3000)

# sudo kill -9 $(sudo lsof -t -i:80)
# sudo pkill -f "node ./node_modules/.bin/nodemon server.js"

# export NODE_EXTRA_CA_CERTS="$(readlink -f ./certificates)/rootCA.pem"

# npm run $1 &
# ./node_modules/.bin/nodemon server.js &
# ./node_modules/.bin/nodemon start_robot_browser.js
# set +x

export NODE_EXTRA_CA_CERTS="/home/hello-robot/hello_robot_ws/src/stretch_teleop_interface/certificates/rootCA.pem"
# cd /home/hello-robot/hello_robot_ws/src/stretch_teleop_interface && npm run localstorage &
cd /home/hello-robot/hello_robot_ws/src/stretch_teleop_interface && pm2 start npm --name="stretch_teleop_interface" -- run localstorage 
cd /home/hello-robot/hello_robot_ws/src/stretch_teleop_interface && pm2 start server.js --watch
cd /home/hello-robot/hello_robot_ws/src/stretch_teleop_interface && pm2 start start_robot_browser.js --watch 
set +x