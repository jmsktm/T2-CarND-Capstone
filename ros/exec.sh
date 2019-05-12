#!/bin/bash

cd ..
> master.log
mkdir -p images
rm images/*

cd ros
catkin_build
source devel/setup.bash
roslaunch launch/styx.launch
