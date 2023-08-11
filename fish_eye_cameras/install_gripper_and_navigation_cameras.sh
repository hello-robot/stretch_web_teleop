#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/hello-robot/teleop_ws/devel/setup.bash

echo ""
echo "Starting gripper and navigation camera installation script."

# UDEV FOR GRIPPER CAMERA
echo ""
echo "Adding udev rules for the gripper camera."
echo "roscd stretch_teleop_interface/fish_eye_cameras/"
roscd stretch_teleop_interface
cd fish_eye_cameras/
echo "sudo cp ./89-hello-gripper-camera.rules /etc/udev/rules.d/"
sudo cp ./89-hello-gripper-camera.rules /etc/udev/rules.d/
echo "sudo udevadm control --reload"
sudo udevadm control --reload
echo "WARNING: You should either reboot the robot or disconnect and reconnect the gripper camera for the new device symlink hello-gripper-camera to take effect."
echo ""

# UDEV FOR NAVIGATION CAMERA
echo ""
echo "Adding udev rules for the navigation camera."
echo "roscd stretch_teleop_interface/fish_eye_cameras/"
roscd stretch_teleop_interface
cd fish_eye_cameras/
echo "sudo cp ./88-hello-navigation-camera.rules /etc/udev/rules.d/"
sudo cp ./88-hello-navigation-camera.rules /etc/udev/rules.d/
echo "sudo udevadm control --reload"
sudo udevadm control --reload
echo "WARNING: You should either reboot the robot or disconnect and reconnect the navigation camera for the new device symlink hello-navigation-camera to take effect."
echo ""

# ROS USB CAMERA PACKAGE
echo ""
echo "Installing ROS USB camera package"
echo "sudo apt-get --yes install ros-noetic-usb-cam"
sudo apt-get --yes install ros-noetic-usb-cam
echo "Done."

# CHANGE UVCVIDEO SETTINGS TO PERMIT BOTH CAMERAS TO WORK AT THE SAME
# TIME AT LOW RESOLUTION (320x240) WITH UNCOMPRESSED YUYV 4:2:2 MODE
echo ""
echo "Change default uvcvideo kernal module settings to enable two quirks: UVC_QUIRK_RESTRICT_FRAME_RATE and UVC_QUIRK_FIX_BANDWIDTH."
echo "roscd stretch_teleop_interface/fish_eye_cameras/"
roscd stretch_teleop_interface
cd fish_eye_cameras/
echo "sudo cp ./uvcvideo.conf /etc/modprobe.d/"
sudo cp ./uvcvideo.conf /etc/modprobe.d/
echo "sudo rmmod uvcvideo"
sudo rmmod uvcvideo
echo "sudo modprobe uvcvideo"
sudo modprobe uvcvideo

echo ""
echo "The gripper and navigation camera installation script has finished."
echo ""
