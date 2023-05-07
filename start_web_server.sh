#!/bin/bash

PORT_NUMBER=5000
lsof -i tcp:${PORT_NUMBER} | awk 'NR!=1 {print $2}' | xargs kill 

export NODE_EXTRA_CA_CERTS="$(readlink -f ./certificates)/rootCA.pem"

sudo --preserve-env ./node_modules/.bin/nodemon server.js &
npm run start