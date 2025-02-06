#!/usr/bin/env python3 
 
import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, 
                            IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    fast_lio_sam_pkg = get_package_share_directory("fast_lio_sam")
    fast_lio_pkg = get_package_share_directory("fast_lio")
    default_sam_config_path = os.path.join(fast_lio_sam_pkg, 'config')
    default_sam_rviz_config_path = os.path.join(
        fast_lio_sam_pkg, 'config', 'sam_rviz.rviz')
    
    default_lio_config_path = os.path.join(fast_lio_pkg, 'config')
    default_lio_rviz_config_path = os.path.join(
        fast_lio_pkg, 'rviz', 'fastlio.rviz')
    default_lio_config_file='avia.yaml'
    
    fast_lio_sam_config_path = LaunchConfiguration('sam_config_path')
    sam_rviz_use = LaunchConfiguration("sam_rviz")
    sam_delay = LaunchConfiguration("sam_delay")
    lio_rviz_use = LaunchConfiguration("lio_rviz")
    fast_lio_sam_rviz_config_path = LaunchConfiguration('sam_rviz_config_path')
    fast_lio_rviz_config_path = LaunchConfiguration('lio_rviz_config_path')
    lio_config_path = LaunchConfiguration('lio_config_path')
    lio_config_file = LaunchConfiguration('lio_config_file')
    # namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
	# <arg name="in_row_method" default="full"/><!--lidar, vision, full--> # TODO remove if possible
    # # nav_camera_front = LaunchConfiguration('nav_camera_front') # TODO set it in configs
    # # nav_camera_front_config = LaunchConfiguration('nav_camera_front_config') # TODO set it in configs
    # # toggle_cam_front = LaunchConfiguration('toggle_cam_front') # TODO to be handled by device manager
    # wayfast_model_path = LaunchConfiguration('wayfast_model_path')

    declare_sam_config_path_cmd = DeclareLaunchArgument(
        name='sam_config_path',
        default_value=default_sam_config_path,
        description="path to fast_lio_sam configs")
    
    declare_lio_config_path_cmd = DeclareLaunchArgument(
        name='lio_config_path',
        default_value=default_lio_config_path,
        description="path to fast_lio configs")
    
    declare_lio_config_file_cmd = DeclareLaunchArgument(
        name='lio_config_file',
        default_value=default_lio_config_file,
        description="fast_lio config filename")
    
    declare_sam_rviz_config_path_cmd = DeclareLaunchArgument(
        name='sam_rviz_config_path',
        default_value=default_sam_rviz_config_path,
        description="path to fast_lio_sam rviz configs")
    
    declare_lio_rviz_config_path_cmd = DeclareLaunchArgument(
        name='lio_rviz_config_path',
        default_value=default_lio_rviz_config_path,
        description="path to fast_lio rviz configs")
    declare_sam_delay_cmd = DeclareLaunchArgument(
        name='sam_delay',
        default_value="5",
        description="Delay after which fast_lio_sam will launch after fast_lio has launched"
    )
    # declare_namespace_cmd = DeclareLaunchArgument(
    #     'namespace',
    #     default_value='tmax',
    #     description='Top-level namespace')
    declare_sam_rviz_cmd = DeclareLaunchArgument(
        "sam_rviz",
        default_value='false',
        choices=["true", "false"],
        description="Use sam rviz")
    
    declare_lio_rviz_cmd = DeclareLaunchArgument(
        "lio_rviz",
        default_value='false',
        choices=["true", "false"],
        description="Use lio rviz")
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value='false',
        choices=["true", "false"],
        description="Use sim time")
    # declare_wayfast_model_path_cmd = DeclareLaunchArgument(
    #     name="wayfast_model_path",
    #     default_value=os.path.join(str(get_package_share_directory\
    #         ("traversability_predictor")), 'checkpoints', 'best_wayfast.pth'),
    #     description="path to wayfast model")
    # yet to add params and launch arguments as given in ros1

    # print_config = launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('config_path'))
    # print_config_sim_time = launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('use_sim_time'))

    fast_lio_sam_group = GroupAction([
        # PushRosNamespace(
        #     namespace=namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(fast_lio_pkg, "launch", "mapping.launch.py")),
            launch_arguments={'use_sim_time': use_sim_time,
                              'config_path': lio_config_path,
                              'config_file': lio_config_file,
                              'rviz': lio_rviz_use,
                              'rviz_cfg': fast_lio_rviz_config_path}.items()),
        ExecuteProcess(
            cmd=["sleep", sam_delay],
            shell=True),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(fast_lio_sam_pkg, "launch", "run_fast_lio_sam.launch.py")),
            launch_arguments={'use_sim_time': use_sim_time,
                              'rviz': sam_rviz_use,
                              'timer_duration': '0',
                              'config_path': fast_lio_sam_config_path,
                              'rviz_cfg': fast_lio_sam_rviz_config_path,
                              'lidar': 'livox'}.items()),
        
    ])

    ld = LaunchDescription()
    ld.add_action(declare_lio_config_file_cmd)
    ld.add_action(declare_lio_config_path_cmd)
    ld.add_action(declare_lio_rviz_cmd)
    ld.add_action(declare_lio_rviz_config_path_cmd)
    ld.add_action(declare_sam_config_path_cmd)
    ld.add_action(declare_sam_delay_cmd)
    ld.add_action(declare_sam_rviz_cmd)
    ld.add_action(declare_sam_rviz_config_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(fast_lio_sam_group)

    return ld