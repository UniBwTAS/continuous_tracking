#!/usr/bin/env sh

# ROS messages for panoptic segmentation
git clone https://github.com/UniBwTAS/object_instance_msgs.git

# visualization plugins (+ dependencies) for rviz
git clone https://github.com/UniBwTAS/ogre_primitives.git
git clone https://github.com/UniBwTAS/rviz_object_instance.git

# actual continuous clustering package
if [ "$1" != "--from-source" ]
then
  git clone https://github.com/UniBwTAS/continuous_clustering.git
fi

# source ROS
. /opt/ros/noetic/setup.sh

# install remaining dependencies in package.xml's from apt & pip repositories
sudo apt install -y -q python3-rosdep
sudo rosdep init || true
sudo apt update
rosdep update
rosdep install --from-paths . --ignore-src -r -y