import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro
import yaml


def generate_launch_description():

    rrbot_description_path = os.path.join(get_package_share_directory('golfinho_model'))

    xacro_file = os.path.join(rrbot_description_path,'urdf','golfinho.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()



    robot_description = {'robot_description': robot_description_config}


    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('golfinho_model'), 'rviz', 'rviz_golfinho_slam.rviz')
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='screen',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description])

    return LaunchDescription([rviz_node])
