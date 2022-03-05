# Copyright 2021 Open Robotics (2021)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition  #new
from launch.substitutions import Command, LaunchConfiguration, PythonExpression #new
import xacro
import yaml


def generate_launch_description():

    # Set the path to different files and folders.
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    # Set the path to the Gazebo ROS package
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    # Set the path to this package.
    pkg_share = FindPackageShare(package='golfinho_model').find('golfinho_model')
 
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 
    # Set the path to the world file
    world_file_name = 'teste.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
   
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    static_map_path = os.path.join(pkg_share, 'maps', 'map_one.yaml')
   
   
   
    # Set the path to the SDF model files.
    
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    default_model_path = os.path.join(pkg_share, 'urdf/golfinho_slam.urdf')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
 
 
    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    slam='True'
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
   
    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')


  #  remappings = [('/tf', 'tf'),
   #            ('/tf_static', 'tf_static')]
 
 
 
 
 
    # Declare the launch arguments 
 
    declare_namespace_cmd = DeclareLaunchArgument(
	 name='namespace',
   	 default_value='',
   	 description='Top-level namespace')
 
    declare_use_namespace_cmd = DeclareLaunchArgument(
  	  name='use_namespace',
  	  default_value='True',
  	  description='Whether to apply a namespace to the navigation stack')
         
    declare_autostart_cmd = DeclareLaunchArgument(
	   name='autostart', 
	   default_value='true',
	   description='Automatically startup the nav2 stack')
 
    declare_bt_xml_cmd = DeclareLaunchArgument(
  	   name='default_bt_xml_filename',
 	   default_value=behavior_tree_xml_path,
 	   description='Full path to the behavior tree xml file to use')
         
    declare_map_yaml_cmd = DeclareLaunchArgument(
	    name='map',
	    default_value=static_map_path,
	    description='Full path to map file to load')
         
    declare_model_path_cmd = DeclareLaunchArgument(
	    name='model', 
	    default_value=default_model_path, 
	    description='Absolute path to robot urdf file')
     
    declare_params_file_cmd = DeclareLaunchArgument(
 	   name='params_file',
 	   default_value=nav2_params_path,
 	   description='Full path to the ROS2 parameters file to use for all launched nodes')
     
    #declare_rviz_config_file_cmd = DeclareLaunchArgument(
 	   #name='rviz_config_file',
 	   #default_value=default_rviz_config_path,
 	   #description='Full path to the RVIZ config file to use')
 
    declare_simulator_cmd = DeclareLaunchArgument(
	   name='headless',
 	   default_value='False',
 	   description='Whether to execute gzclient')
 
    declare_slam_cmd = DeclareLaunchArgument(
	    name='slam',
	    default_value='True',
	    description='Whether to run SLAM')
     
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
	    name='use_robot_state_pub',
	    default_value='True',
	    description='Whether to start the robot state publisher')
 
    declare_use_rviz_cmd = DeclareLaunchArgument(
 	   name='use_rviz',
 	   default_value='True',
 	   description='Whether to start RVIZ')
     
    declare_use_sim_time_cmd = DeclareLaunchArgument(
 	   name='use_sim_time',
 	   default_value='True',
 	   description='Use simulation (Gazebo) clock if true')
 
    declare_use_simulator_cmd = DeclareLaunchArgument(
	   name='use_simulator',
	   default_value='True',
	   description='Whether to start the simulator')
 
    declare_world_cmd = DeclareLaunchArgument(
	   name='world',
           default_value=world_path,
	   description='Full path to the world model file to load')
    
    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
 	   name='gui',
 	   default_value='True',
 	   description='Flag to enable joint_state_publisher_gui')
    

           
    golfinho_description_path = os.path.join(
        get_package_share_directory('golfinho_model'))

    xacro_file = os.path.join(golfinho_description_path,
                              'urdf',
                              'golfinho_slam.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}
   
   
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
      #  remappings=remappings,
        parameters=[robot_description]
    )


    model_address = os.path.join(get_package_share_directory('golfinho_model'),'models','golfinho_slam','model.sdf')

    spawn_entity  = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity','golfinho',
            '-file', model_address,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen',
    )

    # Specify the actions
 
    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
    condition=UnlessCondition(gui),
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher')
    
    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': use_sim_time}])

    
    
     # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
 
    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    
    
      # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    launch_arguments = {'namespace': namespace,
                        'use_namespace': use_namespace,
                        'slam': slam,
                        'map': map_yaml_file,
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'default_bt_xml_filename': default_bt_xml_filename,
                        'autostart': autostart}.items())
 
 
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())
    
    
    start_sync_slam_toolbox_node = Node(
        parameters=[
          os.path.join(pkg_share, 'params', 'nav2_params.yaml'),
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        node_executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    
    rviz2 = IncludeLaunchDescription(PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('golfinho_robsic'), 'launch'), '/golfinho_rviz.launch.py'
           ]),
         )
    
    
    return LaunchDescription([
       # Declare the launch options
       declare_namespace_cmd,
       declare_use_namespace_cmd,
       declare_slam_cmd,
       declare_map_yaml_cmd,
       declare_use_sim_time_cmd,
       declare_params_file_cmd,    
       declare_autostart_cmd,
       declare_bt_xml_cmd,
       #declare_rviz_config_file_cmd,
       declare_use_simulator_cmd,
       declare_use_joint_state_publisher_cmd,
       declare_use_rviz_cmd,
       declare_simulator_cmd,
       declare_world_cmd, 
       
       # Add any conditioned actions
       start_gazebo_server_cmd,
       start_gazebo_client_cmd,
       
       # Add the actions to launch all of the navigation nodes
       rviz2,
       bringup_cmd,


       node_robot_state_publisher,
       spawn_entity,
       start_sync_slam_toolbox_node, #novidade no código
       declare_use_sim_time_cmd,  
       start_robot_localization_cmd, #fora de operação enquanto slam não configurado
      
       # Declare the launch options
    	declare_model_path_cmd,
 	declare_use_robot_state_pub_cmd,
        # Add any actions
    	start_joint_state_publisher_cmd,
    	start_ros2_navigation_cmd
    ])
   
    
