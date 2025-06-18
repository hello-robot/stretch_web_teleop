# Overview

Stretch Web Teleop is an interface, designed for users with motor impairments, to remotely teleoperate a Stretch robot through a web browser using click-based input. It can be set up to teleoperate the robot remotely from anywhere in the world with an internet connection, or simply eyes-off teleop from the next room on a local network. The codebase is built on ROS2, WebRTC, Nav2, and TypeScript.

Recommended: Browse the documentation with MkDocs by running `cd stretch-web-teleop-docs && mkdocs serve`. Ensure [MkDocs](https://www.mkdocs.org/getting-started/) has already been installed on your computer.

## Setup & Installation

The interface is compatible with the Stretch RE1, RE2 and SE3. It currently only supports Ubuntu 22.04 and ROS2 Humble. All Stretch SE3's arrive with the interface pre-installed. For Stretch RE1 and RE2, upgrade your operating system if necessary ([instructions](https://docs.hello-robot.com/0.3/installation/robot_install/)) and create/update the Stretch ROS2 Humble workspace ([instructions](https://docs.hello-robot.com/0.3/installation/ros_workspace/)). This will install all package dependencies and install the web teleop interface.

## Launching the Interface

First, navigate to the folder containing the codebase using:

```
colcon_cd stretch_web_teleop
```

Next, launch the interface and watch for file changes:

```
./launch_interface.sh
```

Alternatively, you can launch the interface with a map with:

```
./launch_interface.sh -m maps/<NAME_OF_MAP>.yaml
```

In the terminal, you will see output similar to:

```
#############################################
LAUNCHING WEB TELEOP
#############################################
Setup environment...
Freeing robot process...
Stopping previous instances...
Reload USB bus...
Start ROS2...
Run webpack...
Start local server...
Start robot browser...

#############################################
DONE! WEB TELEOP IS UP!
Visit the URL(s) below to see the web interface:
https://localhost/operator
https://100.45.161.30/operator
https://192.168.43.12/operator
#############################################
```

URLs with IP addresses that start with `192` can be used if your are on the same WiFi network as the robot. URLs with IP addresses that start with `100` can be used if you are using tailscale to connect to the robot. Visit the appropriate URL in a web browser on your personal laptop or desktop to see the web interface. Ensure your personal computer is connected to the same network as Stretch. You might see a warning that says "Your connection is not private". If you do, click `Advanced` and `Proceed`.

Once you're done with the interface, close the browser and run:

```
./stop_interface.sh
```

**Note:** Only one browser can be connected to the interface at a time.
