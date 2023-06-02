# Overview
This interface enables a user to remotely teleoperate a Stretch robot through a web browser. The interface has been tested using Ubuntu 20.04, ROS Noetic, Python 3.8 and Google Chrome. 

**WARNING: This prototype code and there are security issues. Use this code at your own risk.**

# Setup 
Please make sure you are using Ubuntu 20.04 and have [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) installed before installing the interface.

## Installing the Interface and Simulation Environment 
Create a new catkin workspace and clone [stretch_ros](https://github.com/hello-robot/stretch_ros) (`dev/noetic` branch) and [stretch-web-interface](https://github.com/vinitha910/stretch-web-interface)

If you are installing the interface locally to be run in simulation, clone [hcrl_gazebo](https://github.com/hcrlab/hcrl_gazebo) and [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin):

Run `rosdep install --from-paths . --ignore-src -y -r` in the workspace `src` folder to get all the package dependencies then build and source the workspace. Install   `python3-pcl`:
```
sudo apt-get install python3-pcl
```

Then install the package dependencies for `stretch-web-interface` by running `npm install` in that directory.

Browser features like the camera and microphone access require that the page be running in an SSL context, so we need certificates in order to serve the interface and enable SSL for the rosbridge websocket (see the launch files). We are going to use  [`mkcert`](https://github.com/FiloSottile/mkcert) to manage a set of certificates accross the development machines and Stretch. 

Download the `mkcert` pre-built binaries and install `mkcert`:
```
curl -JLO "https://dl.filippo.io/mkcert/latest?for=linux/amd64"
chmod +x mkcert-v*-linux-amd64
sudo cp mkcert-v*-linux-amd64 /usr/local/bin/mkcert
sudo apt-get install libnss3-tools
rosccd stretch-web-interface/certificates && CAROOT=`pwd` mkcert --install
```

To make your own certificates for your robot/local machine run (this example uses our robot slinky):
```
mkcert slinky.hcrlab.cs.washington.edu slinky.local slinky.dev localhost 127.0.0.1 0.0.0.0 ::1
```
Replace `slinky.hcrlab.cs.washington.edu` and `slinky` with your robot's hostname or IP address. If running locally in simulation, run `mkcert localhost` instead.

Then copy the `Certificate Authority` that `mkcert` created (`rootCA-key.pem` and `rootCA.pem`) on the robot to `~/.local/share/mkcert`

Finally, we need to add the `certfile` and `keyfile` that `mkcert` generated into an environment file for the interface to access. Created a file called `.env` in `stretch-web-interface` with your `certfile` and `keyfile` (this example uses our robot's files):
```
certfile=slinky.hcrlab.cs.washington.edu+6.pem
keyfile=slinky.hcrlab.cs.washington.edu+6-key.pem
```

## Running the Interface in Simulation
Run the following commands in separate terminals:
```
roslaunch hcrl_gazebo house_simulation_stretch.launch
roslaunch stretch-web-interface web_interface_simulation.launch
./start_web_server_and_robot_browser.sh local
```

These commands launch the simulation environment, rosbridge websocket respectively.

Make sure `./start_web_server_and_robot_browser.sh local` is running in the `stretch-web-interface` directory. (`local` allows the site to use the browser's local storage to save useful information. We also have the interface configured with `firebase`.  See firebase [instructions](/src/pages/operator/tsx/storage_handler/README.md)) To use firebase, run `./start_web_server_and_robot_browser.sh firebase` instead. 

Open `localhost/operator` in **google chrome** to see the interface. You might see a `Webpage not secure` warning, click advanced and proceed. 

## Running the Interface on Stretch
Run the following commands in separate terminals **on the robot**:
```
roslaunch stretch-web-interface web_interface.launch
./start_web_server_and_robot_browser.sh local
```
Make sure `./start_web_server_and_robot_browser.sh local` is running in the `stretch-web-interface` directory. (`local` allows the site to use the browser's local storage to save useful information. We also have the interface configured with `firebase`. See firebase [instructions](/src/pages/operator/tsx/storage_handler/README.md)). To use firebase, run `./start_web_server_and_robot_browser.sh firebase` instead. 

Enter the robot's hostname or IP address followed by `/operator` in **google chrome** to open the interface. For example, to open up our robot's interface, we would open `slinky.hcrlab.cs.washington.edu/operator` in our browser. You might see a `Webpage not secure` warning, click advanced and proceed. 
