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

    #New
    # Set the path to the Gazebo ROS package
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    # Set the path to this package.
    pkg_share = FindPackageShare(package='golfinho_model').find('golfinho_model')
 
    # Set the path to the world file
    world_file_name = 'teste.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
   
    # Set the path to the SDF model files.
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
 
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
 
    declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
     
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
 
 
    rrbot_description_path = os.path.join(get_package_share_directory('golfinho_model'))
    xacro_file = os.path.join(rrbot_description_path,'urdf','golfinho_cartographer.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()    
    
    robot_description = {'robot_description': robot_description_config}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description,{'use_sim_time': LaunchConfiguration('use_sim_time', default='True')}],
        arguments=[xacro_file]
    )


#spawn

    model_address = os.path.join(get_package_share_directory('golfinho_model'),'models','golfinho_cartographer','model.sdf')

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

     # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
 
    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    
    cartographer = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('golfinho_cartographer'), 'launch'), '/golfinho_cartographer.launch.py']))


    return LaunchDescription([
      node_robot_state_publisher,
      spawn_entity,
      cartographer,
      declare_simulator_cmd,
      declare_use_sim_time_cmd,
      declare_use_simulator_cmd,
      declare_world_cmd,
      start_gazebo_server_cmd,
      start_gazebo_client_cmd,
    ])
    
    
    
    
    #transform = Node ( 
    #        package = 'tf2_ros', 
    #        executable = 'static_transform_publisher', 
    #        name="map_to_odom",
    #        arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"])
            
            #['--x', '0', '--y', '0', '--z ',' 0 ',' --yaw ',' 0 ',' --pitch ',' 0 ',' --roll ',' 0 ',' --frame-id ',' map ',' - -child-frame-id ',' odom'] )
    
    
