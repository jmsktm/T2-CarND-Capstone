1. Clone repository
2. sudo apt-get update
sudo apt-get install -y ros-kinetic-dbw-mkz-msgs
cd /home/workspace/T2-CapstoneProject/ros
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
3. pip install catkin-pkg-modules (Have to run this twice at least)
4. catkin_make
5. source devel/setup.bash
6. roslaunch launch/styx.launch
