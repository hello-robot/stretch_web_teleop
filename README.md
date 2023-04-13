## Installing interface and simulation environment

Create a new catkin workspace and clone the following packages:
```
git clone https://github.com/hcrlab/hcrl_gazebo.git
git clone https://github.com/hcrlab/stretch_ros.git
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
```

Run `rosdep install --from-paths . --ignore-src -y -r` in the workspace `src` folder to get all the package dependencies then build the workspace.

Then install the package dependencies for `stretch-web-interface` but running `npm install` in that directory.

## Running interface in simulation environment
Run the following commands in separate terminals:
```
roslaunch hcrl_gazebo house_simulation_stretch.launch
roslaunch stretch-web-interface web_interface_simulation.launch
rosrun web_video_server web_video_server
```

These commands launch the simulation environment, rosbridge websocket and stream videos respectively.

Run `npm run start` in the `stretch-web-interface` directory. A webpage will automatically open and you will see all video streams from navigation camera, realsense and gripper camera.

## Running the interface on the real robot
Run the following commands in separate terminals **on the robot**:
```
roslaunch stretch-web-interface web_interface_simulation.launch
npm run start
```

On your **local machine** configure your `ROS_IP` and `ROS_MASTER_URI` and run `rosrun web_video_server web_video_server`. If your `ROS_IP` and `ROS_MASTER_URI` are not configured correctly you will no be able to see the video streams. 

Enter the robot's IP address on your web browser. (TODO: make it work with aliases) 
