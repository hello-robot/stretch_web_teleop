## Setting up simulation environment

Create a new catkin workspace and clone the following packages:
```
git clone https://github.com/hcrlab/hcrl_gazebo.git
git clone https://github.com/hcrlab/stretch_ros.git
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
git clone https://github.com/vinitha910/stretch-web-interface.git
```

Run `rosdep install --from-paths . --ignore-src -y -r` from the workspace `src` folder to get all the package dependencies then build the workspace.

Then install the package dependencies for `stretch-web-interface` but running `npm install` in that directory.

