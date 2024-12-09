#!/usr/bin/env python3 
 
import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def load_yaml_file(yaml_file_path):
    try:
        with open(yaml_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError as err: # parent of IOError, OSError *and* WindowsError where available
        print("Could not load YAML file. Error: ", err)
        return None 


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path', default= \
        os.path.join(get_package_share_directory("fast_lio_sam"), "config"))
    config_path_value = config_path.perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    params_file = os.path.join(
        config_path_value,
        "config_2.yaml"
    )

    params = load_yaml_file(params_file)

    fast_lio_sam_params = params["fast_lio_sam_node"]["ros__parameters"]
    
    fast_lio_sam_node = Node(
        package="fast_lio_sam",
        executable="fast_lio_sam_node",
        name="fast_lio_sam_node",
        parameters=fast_lio_sam_params,
        output="screen"
    )

    return[
        fast_lio_sam_node
    ]

def generate_launch_description():
    

    ld = LaunchDescription()
    
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld