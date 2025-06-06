#!/bin/bash

sudo apt-get install -y python3-pip python3-pykdl
sudo apt-get install -y ament-cmake
sudo apt-get install -y ros-humble-ros2-control ros-humble-joint-state-publisher-gui ros-humble-ros2-controllers ros-humble-xacro ros-humble-rmw-cyclonedds-cpp ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control  ros-humble-tf-transformations ros-humble-hardware-interface ros-humble-controller-interface ros-humble-realtime-tools ros-humble-vision-msgs ros-humble-rviz2 ros-humble-compressed-image-transport ros-humble-foxglove-bridge
sudo apt-get install -y libclang-dev libudev-dev pkg-config unzip

# dependencies for Orbbec camera
sudo apt-get install -y usbutils nlohmann-json3-dev ros-humble-image-transport ros-humble-image-publisher ros-humble-camera-info-manager ros-humble-diagnostic-updater ros-humble-diagnostic-msgs ros-humble-statistics-msgs
