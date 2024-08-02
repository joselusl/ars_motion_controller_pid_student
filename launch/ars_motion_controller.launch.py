#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare





def generate_launch_description():
    # Define the arguments
    screen_arg = DeclareLaunchArgument(
        'screen', default_value='screen',
        description='Output setting for the nodes'
    )

    ars_motion_controller_node_name_arg = DeclareLaunchArgument(
        'ars_motion_controller_node_name', default_value='ars_motion_controller_node',
        description='Name of the node'
    )

    ars_motion_controller_yaml_file_arg=DeclareLaunchArgument(
      'config_param_motion_controller_yaml_file',
      default_value=PathJoinSubstitution(['config_motion_controller_pid.yaml']), 
      description='Path to the config_param_motion_controller_yaml_file'
    )

    robot_pose_topic_arg = DeclareLaunchArgument(
      'robot_pose_topic', default_value='/estim_robot_pose',
      description='Topic robot_pose')

    robot_velocity_world_topic_arg = DeclareLaunchArgument(
      'robot_velocity_world_topic', default_value='/estim_robot_velocity_world',
      description='Topic robot_velocity_world')

    robot_cmd_ctr_unstamped_topic_arg = DeclareLaunchArgument(
        'robot_cmd_ctr_unstamped_topic', default_value='/robot_cmd_unstamped',
        description='Topic robot_cmd_ctr_unstamped')
    
    robot_cmd_ctr_stamped_topic_arg = DeclareLaunchArgument(
        'robot_cmd_ctr_stamped_topic', default_value='/robot_cmd_stamped',
        description='Topic robot_cmd_ctr_stamped')
      
    robot_pose_ref_topic_arg = DeclareLaunchArgument(
          'robot_pose_ref_topic', default_value='/robot_pose_ref',
          description='Topic robot_pose_ref')
    
    robot_velocity_world_ref_topic_arg = DeclareLaunchArgument(
          'robot_velocity_world_ref_topic', default_value='/robot_velocity_world_ref',
          description='Topic robot_velocity_world_ref')
    
    robot_cmd_ref_topic_arg = DeclareLaunchArgument(
          'robot_cmd_ref_topic', default_value='/robot_cmd_ref',
          description='Topic robot_cmd_ref')


    # Get the launch configuration for parameters
    ars_motion_controller_conf_yaml_file = PathJoinSubstitution([FindPackageShare('ars_motion_controller_pid'), 'config', LaunchConfiguration('config_param_motion_controller_yaml_file')])
    

    # Define the nodes
    ars_motion_controller_node = Node(
        package='ars_motion_controller_pid',
        executable='ars_motion_controller_ros_node',
        name=LaunchConfiguration('ars_motion_controller_node_name'),
        output=LaunchConfiguration('screen'),
        parameters=[{'config_param_motion_controller_pid_yaml_file': ars_motion_controller_conf_yaml_file}],
        remappings=[
          ('robot_cmd_ctr', LaunchConfiguration('robot_cmd_ctr_unstamped_topic')),
          ('robot_cmd_ctr_stamped', LaunchConfiguration('robot_cmd_ctr_stamped_topic')),
          ('robot_pose', LaunchConfiguration('robot_pose_topic')),
          ('robot_velocity_world', LaunchConfiguration('robot_velocity_world_topic')),
          ('robot_pose_ref', LaunchConfiguration('robot_pose_ref_topic')),
          ('robot_velocity_world_ref', LaunchConfiguration('robot_velocity_world_ref_topic')),
          ('robot_cmd_ref', LaunchConfiguration('robot_cmd_ref_topic')),
        ]
    )



    return LaunchDescription([
        screen_arg,
        ars_motion_controller_node_name_arg,
        ars_motion_controller_yaml_file_arg,
        robot_pose_topic_arg,
        robot_velocity_world_topic_arg,
        robot_cmd_ctr_unstamped_topic_arg,
        robot_cmd_ctr_stamped_topic_arg,
        robot_pose_ref_topic_arg,
        robot_velocity_world_ref_topic_arg,
        robot_cmd_ref_topic_arg,
        ars_motion_controller_node,
    ])
