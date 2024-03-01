# Overview
This interface enables a user to remotely teleoperate a Stretch robot through a web browser. This website can be set up to teleoperate the robot remotely from anywhere in the world with an internet connection, or simply eyes-off teleop from the next room on a local network. The codebase is built on ROS2, WebRTC, Nav2, and TypeScript.

# Setup
The interface is compatible with the Stretch RE1, RE2 and SE3. It currently only supports Ubuntu 22.04 and ROS2 Humble. Upgrade your operating system if necessary ([instructions]()) and create a the Stretch ROS2 Humble workspace ([instructions]()). This will install all package dependencies and install the web teleop interface.  

## Installing Teleop Cameras

# Launching the Interface

First, navigate to the folder containing the codebase using:

```
colcon_cd stretch_web_teleop
```

Next, launch the interface:
```
./launch_interface
```

If you'd like to launch the interface with a map run:
```
./launch_interface -m maps/<NAME_OF_MAP>.yaml
```

In the terminal, you will see output similar to:
```
Visit the URL(s) below to see the web interface:
https://localhost/operator
https://192.168.1.14/operator
```

Look for a URL like `https://<ip_address>/operator`. Visit this URL in a web browser on your personal laptop or desktop to see the web interface. Ensure your personal computer is connected to the same network as Stretch. You might see a warning that says "Your connection is not private". If you do, click `Advanced` and `Proceed`. 

Once you're done with the interface, close the browser and run:
```
./stop_interface.sh
```

**Note:** Only one browser can be connected to the interface at a time.

# Using the Interface Remotely

**WARNING: This is prototype code and there are security issues. Deploy this code at your own risk.**

We recommend setting up the interface for remote use using [ngrok](https://ngrok.com/docs/what-is-ngrok/). First, create an account with `ngrok` and follow the Linux installation instructions in the `Setup & Installation` tab in your ngrok account dashboard. 

Navigate to the `Domains` tab and click `Create Domain`. ngrok will automatically generate a domain name for your free account. You will see a domain similar to `deciding-hornet-purely.ngrok-free.app`. Follow the interface launch instructions and then start the ngrok tunnel by running the following command (replace `<NGROK_DOMAIN>` with your account's domain and `user:password` with a secure username and password):

```
ngrok http --basic-auth="user:password" --domain=<NGROK_DOMAIN> 443
```

In your browser, open `https://<NGROK_DOMAIN>/operator` to see the interface. You will then be prompted to enter the appropiate username and password. Note, anyone in the world with internet access can open this link. 

## Storing Ngrok Tunnel Configuration 
To store this configuration, open the ngrok config file:
```
ngrok config edit
```

Add the following configuration to the file. Make sure to update `<NGROK_AUTH_TOKEN>`, `<NGROK_DOMAIN>`, and `admin:password` with the appropiate values.
```
authtoken: <NGROK_AUTH_TOKEN>
version: 2
tunnels:
    stretch-web-teleop:
        proto: http
        domain: <NGROK_DOMAIN>
        addr: 443
        basic_auth: 
          - "admin:password"
        host_header: rewrite
        inspect: true
```

Now run `ngrok start stretch-web-teleop` to start the tunnel and navigate to `https://<NGROK_DOMAIN>/operator`. You will then be prompted to enter the appropiate username and password. 

# Usage Guide

TODO 


# Troubleshooting

TODO


# Licenses
The following license applies to the contents of this directory written by Vinitha Ranganeni, Noah Ponto, authors associated with the University of Washington, and authors associated with Hello Robot Inc. (the "Contents"). This software is intended for use with Stretch ® mobile manipulators produced and sold by Hello Robot ®.  

Copyright 2023 Vinitha Ranganeni, Noah Ponto, the University of Washington, and Hello Robot Inc.

The Contents are licensed under the Apache License, Version 2.0 (the "License"). You may not use the Contents except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0 

Unless required by applicable law or agreed to in writing, the Contents are distributed under the License are distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

============================================================

Some of the contents of this directory derive from the following repositories: 

https://github.com/hello-robot/stretch_web_interface

https://github.com/hcrlab/stretch_web_interface

https://github.com/hcrlab/stretch_teleop_interface

Text from relevant license files found in these repositories.
