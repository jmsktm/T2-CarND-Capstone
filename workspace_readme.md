## Setup for a non-docker system

sudo apt-get install software-properties-common  
sudo add-apt-repository ppa:git-core/ppa  
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash  
sudo apt-get install git-lfs  
git lfs install  
git clone https://github.com/jmsktm/T2-CarND-Capstone  
cd /home/workspace/T2-CarND-Capstone/ros  
rm -rf build devel  
sudo apt-get update  
sudo apt-get install -y ros-kinetic-dbw-mkz-msgs  
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y  
pip install catkin-pkg-modules # (Have to run this twice at least)  
catkin_make  
source devel/setup.bash  
roslaunch launch/styx.launch  
