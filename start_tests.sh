#!/bin/bash

# PORT_NUMBER=3000
# lsof -i tcp:${PORT_NUMBER} | awk 'NR!=1 {print $2}' | xargs kill 
# kill $(lsof -t -i:3000)

sudo kill -9 $(sudo lsof -t -i:80)
sudo pkill -f "node ./node_modules/.bin/nodemon server.js"

export NODE_EXTRA_CA_CERTS="$(readlink -f ./certificates)/rootCA.pem"

npm run test &
sudo --preserve-env ./node_modules/.bin/nodemon server.js &
./node_modules/.bin/nodemon start_robot_browser.js
set +x
