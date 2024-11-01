#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    
    return LaunchDescription([
        # Declare use_sim_time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Launch rviz_goal_relay node
        Node(
            package='asl_tb3_lib',
            executable='rviz_goal_relay.py',
            parameters=[{'output_channel': '/cmd_nav'}],
        ),
        
        # Launch state_publisher node
        Node(
            package='asl_tb3_lib',
            executable='state_publisher.py',
        ),
        
        # Launch your navigator node
        Node(
            package='autonomy_repo',
            executable='navigator.py',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # Include RViz launch file
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('asl_tb3_sim'), 'launch', 'rviz.launch.py']
            ),
            launch_arguments={
                'config': PathJoinSubstitution(
                    [FindPackageShare('autonomy_repo'), 'rviz', 'default.rviz']
                ),
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])