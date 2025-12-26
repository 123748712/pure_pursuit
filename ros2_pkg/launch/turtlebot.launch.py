import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare(package='turtlebot3_gazebo').find('turtlebot3_gazebo')
    pkg_navigation_ros = FindPackageShare(package='turtlebot3_navigation2').find('turtlebot3_navigation2')

    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'turtlebot3_yolo.launch.py')
        )
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='True',
        description='use_sim_time_arg'
    )

    map_arg = DeclareLaunchArgument(
        'map', 
        default_value= os.path.join(os.environ['HOME'], 'map', 'yolo_suv.yaml'),
        description='map_arg'
    )

    nav2_params = PathJoinSubstitution([
        FindPackageShare('turtlebot3_navigation2'),
        'param',
        'humble',
        'waffle_pi.yaml'
    ])


    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')

    

    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation_ros, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': nav2_params
        }.items()
    )


    img_publisher_node = Node(
        package='ros2_pkg',
        executable='img_publisher_node',
        name='img_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    img_subscriber_node = Node(
        package='ros2_pkg',
        executable='img_subscriber_node',
        name='img_subscriber_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )


    return LaunchDescription([
        start_gazebo_cmd,
        use_sim_time_arg,
        map_arg,
        start_navigation_cmd,
        img_publisher_node,
        img_subscriber_node
    ])

