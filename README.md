## Installing interface and simulation environment

Create a new catkin workspace and clone the following packages:
```
git clone https://github.com/hcrlab/hcrl_gazebo.git
git clone https://github.com/hcrlab/stretch_ros.git
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
git clone https://github.com/vinitha910/stretch-web-interface.git
```

Run `rosdep install --from-paths . --ignore-src -y -r` in the workspace `src` folder to get all the package dependencies then build and source the workspace.

Then install the package dependencies for `stretch-web-interface` but running `npm install` in that directory.

Browser features like the camera and microphone access require that the page be running in an SSL context, so we need certificates in order to serve the interface and enable SSL for the rosburdge websocket (see the launch files). We are going to use  [`mkcert`](https://github.com/FiloSottile/mkcert) to manage a set of certificates accross the development machines and Stretch. 

Download the `mkcert` pre-built binaries and install `mkcert`:
```
curl -JLO "https://dl.filippo.io/mkcert/latest?for=linux/amd64"
chmod +x mkcert-v*-linux-amd64
sudo cp mkcert-v*-linux-amd64 /usr/local/bin/mkcert
sudo apt-get install libnss3-tools
rosccd stretch-web-interface/certificates && CAROOT=`pwd` mkcert --install
```

To make your own certificates for your robot run (this example uses our robot slinky):
```
mkcert slinky.hcrlab.cs.washington.edu slinky.local slinky.dev localhost 127.0.0.1 0.0.0.0 ::1
```

Then copy the `Certificate Authority` that `mkcert` created (`rootCA-key.pem` and `rootCA.pem`) on the robot to `~/.local/share/mkcert`

To install the interface on the robot, follow the same instructions above but do not clone `hcrl_gazebo` or `realsense_gazebo_plugin`.

## Running interface in simulation environment
Run the following commands in separate terminals:
```
roslaunch hcrl_gazebo house_simulation_stretch.launch
roslaunch stretch-web-interface web_interface_simulation.launch
```

These commands launch the simulation environment, rosbridge websocket respectively.

Run `npm run start` in the `stretch-web-interface` directory. Open `localhost:3000` in your web browser to see the interface.

## Running the interface on the real robot
Run the following commands in separate terminals **on the robot**:
```
roslaunch stretch-web-interface web_interface.launch
npm run start 
```
Make sure `npm run start` is running in the `stretch-web-interface` directory

On your **local machine** configure your `ROS_IP` to you IP address and the `ROS_MASTER_URI` to the robot's ros master. If your `ROS_IP` and `ROS_MASTER_URI` are not configured correctly you will no be able to see the video streams. 

Run `rosrun web_video_server web_video_server` on your **local machine**.

Enter the robot's IP address in your web browser to open the interface.
