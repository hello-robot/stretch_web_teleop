# Instructions for using the feature/nodejs branch

This guide contains instructions for getting started with the feature/nodejs branch on the Stretch Web Teleop repo. This file should be deleted before the PR is merged, and the instructions should become part of the [default installation scripts](https://github.com/hello-robot/stretch_install/).

## Setup

1. Follow the [quickstart instructions here](https://github.com/hello-robot/stretchpy/?tab=readme-ov-file#quickstart).

1. Stretch Web Teleop should be located at `~/ament_ws/src/stretch_web_teleop` on the robot. Use git to check out the feature/nodejs branch.

1. Delete `node_modules` and regenerate it using:

```
npm install
```

4. Ensure certificates are in the `~/ament_ws/src/stretch_web_teleop/certificates/` folder and that `.env` points to them.

## Running / Developing

1. Compile frontend (recompile whenever frontend is changed)

```
cd ~/ament_ws/src/stretch_web_teleop
npm --name="stretch_web_teleop" -- run localstorage
```

Hit Ctrl+C once the TypeScript build finishes.

2. Run the frontend server. Be sure to change the CERT path in the command below.

```
export NODE_EXTRA_CA_CERTS="/home/hello-robot/ament_ws/src/stretch_web_teleop/certificates/rootCA.pem"
node server.js
```

3. In a new terminal, run the robot server

```
python3 -m stretch.serve
```

4. In a new terminal, run the backend. Be sure to change the CERT path in the command below.

```
export NODE_EXTRA_CA_CERTS="/home/hello-robot/ament_ws/src/stretch_web_teleop/certificates/rootCA.pem"
npx ts-node src/backend/main.ts
```

5. Open up a browser, preferably Chrome, and go to [https://localhost/operator](https://localhost/operator)

If you ever see the operator screen go white (and errors in the console), go to console and enter `localStorage.clear()`
