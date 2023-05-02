#!/bin/bash

export NODE_EXTRA_CA_CERTS="$(readlink -f ./certificates)/rootCA.pem"

sudo --preserve-env ./node_modules/.bin/nodemon server.js &
npm run start