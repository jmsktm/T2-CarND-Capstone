#!/bin/bash

catkin_build
source devel/setup.bash
roslaunch launch/styx.launch
