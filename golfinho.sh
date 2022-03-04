rm -rf build log install
source /opt/ros/foxy/setup.bash
colcon build
. install/setup.bash

ros2 launch golfinho_robsic golfinho_gazebo_cartograph.launch.py

