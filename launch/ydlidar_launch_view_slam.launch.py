#!/usr/bin/python3
# Copyright 2020, EAIBOT
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    rviz_config_file = os.path.join(share_dir, 'config','ydlidar.rviz')
    parameter_file = LaunchConfiguration('params_file')
    # name = 'ydlidar_ros2_driver_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'ydlidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    # matcher_parameter_file = LaunchConfiguration('params_file')
    # name = 'ydlidar_ros2_driver_node'

    # matcher_params_declare = DeclareLaunchArgument('params_file',
    #                                        default_value=os.path.join(
    #                                            share_dir, 'params', 'matcher_param.yaml'),
    #                                        description='FPath to the ROS2 parameters file to use.')

    driver_node = Node(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',
                                )

# <node pkg ="tf" type="static_transform_publisher" name="map_to_odom" args="0.0 0.0 0.0 0.0 0.0 0.0 /map /nav 40"/>

#      <node pkg ="tf" type="static_transform_publisher" name="odom_to_base_link" args="0.0 0.0 0.0 0.0 0.0 0.0 /nav /base_footprint 40"/>

#      <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser" args="0.2245 0.0 0.2 0.0 0.0 0.0 /base_footprint /laser_frame 40" />

    map_to_odom_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='map_to_odom',
                    arguments=['0.0', '0.0', '0.0','0.0', '0.0', '0.0', '1','map','nav'],
                    )
    odom_to_base_link_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='odom_to_base_link',
                    arguments=['0.0', '0.0', '0.0','0.0', '0.0', '0.0', '1','nav','base_link'],
                    )
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                    )
    imu_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='imu_node',
                    arguments=['0', '0', '0.0','0', '0', '0', '1','base_link','imu_link'],
                    )
    rviz2_node = Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    )
    gmapping_node = Node(package='slam_gmapping',
                    executable='slam_gmapping',
                    name='slam_gmapping',
                    parameters=[{'use_sim_time':True}],
                    )

    # launch file for cartographer
    # cartographer_dir = get_package_share_directory('turtlebot3_cartographer')

    # included_launch2 = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #                 cartographer_dir + '/launch/cartographer.launch.py'), 

    #                 # some arguments for launch
    #                 launch_arguments={
    #                     "use_sim_time": "True"  # "arguments(param) name":"value" 
    #                 }.items())
    rf2o_dir = get_package_share_directory('rf2o_laser_odometry')

    included_launch1 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    rf2o_dir + '/launch/rf2o_laser_odometry.launch.py'), 

                    # some arguments for launch
                    launch_arguments={
                        "use_sim_time": "True"  # "arguments(param) name":"value" 
                    }.items())
    

    # gmapping_dir = get_package_share_directory('slam_gmapping')

    # included_launch2 = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #                 gmapping_dir + '/launch/slam_gmapping.launch.py'), 

    #                 # some arguments for launch
    #                 launch_arguments={
    #                     "use_sim_time": "True"  # "arguments(param) name":"value" 
    #                 }.items())

    return LaunchDescription([
        params_declare,
        driver_node,
        map_to_odom_node,
        odom_to_base_link_node,
        # tf2_node,
        included_launch1,
        # gmapping_node,
        rviz2_node
        # read_launch
    ])
