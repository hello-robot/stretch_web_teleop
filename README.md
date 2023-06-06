# Table of Contents 
- [Overview](#overview)
- [Setup](#setup)
    - [Installing the Interface and Simulation Environment](#installing-the-interface-and-simulation-environment)
    - [Setup Fisheye Cameras](#setup-fisheye-cameras)
    - [Certificates](#certificates)
        - [Creating Certificates for your Robot](#creating-certificates-for-your-robot)
        - [Creating Certificates for Simulation](#creating-certificates-for-simulation)
        - [Adding Certificates to Environment File](#adding-certificates-to-environment-file)
- [Running the Interface](#running-the-interface)
    - [Running the Interface in Simulation](#running-the-interface-in-simulation)
    - [Running the Interface on Stretch](#running-the-interface-on-stretch)

# Overview
This interface enables a user to remotely teleoperate a Stretch robot through a web browser. The interface has been tested using Ubuntu 20.04, ROS Noetic, Python 3.8 and Google Chrome. 

**WARNING: This prototype code and there are security issues. Use this code at your own risk.**

# Setup 
Please make sure you are using Ubuntu 20.04 and have [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) installed before installing the interface.

## Installing the Interface and Simulation Environment 
Create a new catkin workspace, and within the `src` directory clone [stretch_ros](https://github.com/hello-robot/stretch_ros) (`dev/noetic` branch) and [stretch_teleop_interface](https://github.com/vinitha910/stretch-web-interface)

If you are installing the interface locally to be run in simulation, clone [hcrl_gazebo](https://github.com/hcrlab/hcrl_gazebo) and [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin):

From the `src` directory, get all the package dependencies by running:
```
rosdep install --from-paths . --ignore-src -y -r
```
Then build and source the workspace:
```
catkin build
source devel/setup.bash
```
Install   `python3-pcl`:
```
sudo apt-get install python3-pcl
```

Then install the package dependencies for `stretch_teleop_interface` by running `npm install` in that directory.

## Setup Fisheye Cameras
The interface makes use of Stretch's [teleop kit](https://hello-robot.com/stretch-teleop-kit) which contains two fish-eye cameras. We need to setup the `udev` rules so Stretch can identify the cameras.

> **_NOTE_**: It is possibile to customize the interface to only use the Realsense but the fish-eye cameras add additional perspective making it easier to teleoperate the robot. 

Begin by sourcing your workspace, if you haven't already, and run `./install_gripper_and_navigation_cameras.sh` in `stretch_teleop_interface/fish_eye_cameras`. 

Run `sudo dmesg | grep usb` to get information about currently plugged in devices. If you unplug and replug one of the fish-eye cameras before running the command its information will be displayed at the bottom. You should see something like this: 
```
usb 1-2.1.1: New USB device found, idVendor=1d6c, idProduct=0103, bcdDevice= 0.10
```

Update the `idVendor` and `idProduct` in `/etc/udev/rules.d/88-hello-navigation-camera.rules` and `/etc/udev/rules.d/89-hello-gripper-camera.rules`. In this example, `1-2.1.1` should be the `KERNELS` in the `udev` rules. In this example, our `udev` rules would look like this:
```
KERNEL=="video*", KERNELS=="1-2.*" ATTRS{idVendor}=="1d6c", ATTRS{idProduct}=="0103", MODE:="0777", SYMLINK+="hello-navigation-camera" 
```

Unplug and replug both cameras after updating the `udev` rules and reload the `udev` rules: `sudo udevadm control --reload-rules`

Run `ll /dev/hello*` and check for both `/dev/hello-navigation-camera` and `/dev/hello-gripper-camera`. If you don’t see them then try unplugging/replugging the cameras a couple times and/or restarting the robot.

> **_NOTE_**: Every time you change the `udev` rules you will have to reload the rules and unplug and replug the cameras to see if the symlinks appear.

## Certificates

Browser features like the camera and microphone access require that the page be running in an SSL context, so we need certificates in order to serve the interface and enable SSL for the rosbridge websocket (see the launch files). We are going to use  [`mkcert`](https://github.com/FiloSottile/mkcert) to manage a set of certificates accross the development machines and Stretch. 

Download the `mkcert` pre-built binaries and install `mkcert`:
```
curl -JLO "https://dl.filippo.io/mkcert/latest?for=linux/amd64"
chmod +x mkcert-v*-linux-amd64
sudo cp mkcert-v*-linux-amd64 /usr/local/bin/mkcert
sudo apt-get install libnss3-tools
roscd stretch_teleop_interface/certificates && CAROOT=`pwd` mkcert --install

```
The commands above have executed properly when there are two `.pem` files in the `stretch_teleop_interface/certificates` directory. These will likely be named `rootCA.pem` and `rootCA-key.pem`.
> **_NOTE_**: You may need to double check the final command executed properly. You may need to navigate to the main directory of your catkin workspace and run `source devel/setup.bash` in order for `roscd` to locate the `stretch_teleop_interface/certificates` directory. You can also manually navigate to that directory and run ``CAROOT=`pwd` mkcert --install``.

Then copy the Certificate Authority that `mkcert` created (`rootCA-key.pem` and `rootCA.pem`) on the robot to `~/.local/share/mkcert`

### Creating Certificates for your Robot
To make your own certificates for your robot run (this example uses our robot slinky):
```
mkcert slinky.hcrlab.cs.washington.edu slinky.local slinky.dev localhost 127.0.0.1 0.0.0.0 ::1
```
Replace `slinky.hcrlab.cs.washington.edu` and `slinky` with your robot's hostname or IP address.

### Creating Certificates for Simulation
To make certificates for running locally in simulation run:
```
mkcert localhost
```

> **_NOTE_**: Whether working on a real robot or in simulation, at this point you should have four different (two pairs of) `.pem` files in the `certificates` directory.


### Adding Certificates to Environment File
Finally, we need to add the files generated by `mkcert` into an environment file for the interface to access. Create a file named `.env` in `stretch_teleop_interface` with your `certfile` and `keyfile`:
```
# Contents of the stretch_teleop_interface/.env:

# Example for robot certificates
certfile=slinky.hcrlab.cs.washington.edu+6.pem
keyfile=slinky.hcrlab.cs.washington.edu+6-key.pem

# Example for local simulation certificates
certfile=localhost.pem
keyfile=localhost-key.pem
```

# Running the Interface

## Running the Interface in Simulation
Run the following commands in separate terminals:

Launch simulation:
```
roslaunch hcrl_gazebo house_simulation_stretch.launch
```

Launch the backend:
```
roslaunch stretch_teleop_interface web_interface_simulation.launch \
certfile:=localhost.pem \
keyfile:=localhost-key.pem 
```

From within the `stretch_teleop_interface` directory, start the server and robot browser:
```
./start_web_server_and_robot_browser.sh local
```

> **_NOTE_**: Make sure `./start_web_server_and_robot_browser.sh local` is running in the `stretch_teleop_interface` directory. (`local` allows the site to use the browser's local storage to save useful information. We also have the interface configured with `firebase`.  See firebase [instructions](/src/pages/operator/tsx/storage_handler/README.md)) To use firebase, run `./start_web_server_and_robot_browser.sh firebase` instead. 

Open `localhost/operator` in **google chrome** to see the interface. You might see a `Webpage not secure` warning, click advanced and proceed. 

## Running the Interface on Stretch
Run the following commands in separate terminals **on the robot** (remember to change the `certfile` and `keyfile`):
```
roslaunch stretch_teleop_interface web_interface.launch \
certfile:=slinky.hcrlab.cs.washington.edu.pem \
keyfile:=slinky.hcrlab.cs.washington.edu-key.pem 
```

From within the `stretch_teleop_interface` directory, start the server and robot browser:
```
./start_web_server_and_robot_browser.sh local
```
Make sure `./start_web_server_and_robot_browser.sh local` is running in the `stretch_teleop_interface` directory. (`local` allows the site to use the browser's local storage to save useful information. We also have the interface configured with `firebase`. See firebase [instructions](/src/pages/operator/tsx/storage_handler/README.md)). To use firebase, run `./start_web_server_and_robot_browser.sh firebase` instead. 

Enter the robot's hostname or IP address followed by `/operator` in **google chrome** to open the interface. For example, to open up our robot's interface, we would open `slinky.hcrlab.cs.washington.edu/operator` in our browser. You might see a `Webpage not secure` warning, click advanced and proceed. 