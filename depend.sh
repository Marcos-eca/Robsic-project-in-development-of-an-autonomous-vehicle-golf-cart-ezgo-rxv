source /opt/ros/foxy/setup.bash

sudo apt install ros-foxy-turtlebot3*

export TURTLEBOT3_MODEL=burger

export GAZEBO_MODEL_PATH=`ros2 pkg \
prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/
