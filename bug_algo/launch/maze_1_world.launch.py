#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool, roeunsea

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    bug2_nav_pkg = get_package_share_directory('bug2_nav')
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # World file from bug2_nav package
    world = os.path.join(bug2_nav_pkg, 'worlds', 'turtlebot3_maze1.world')

    # Set GAZEBO_MODEL_PATH to include bug2_nav and turtlebot3_gazebo models
    bug2_model_path = os.path.join(bug2_nav_pkg, 'models')
    turtlebot3_model_path = os.path.join(turtlebot3_gazebo_pkg, 'models')
    os.environ['GAZEBO_MODEL_PATH'] = f"{bug2_model_path}:{turtlebot3_model_path}:{os.environ.get('GAZEBO_MODEL_PATH', '')}"

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='0.0', description='Initial x position of TurtleBot3')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='0.0', description='Initial y position of TurtleBot3')

    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_pkg, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn TurtleBot3
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_pkg, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Bug2 navigation node
    bug2_nav_node = Node(
        package='bug2_nav',
        executable='bug2_nav_node',
        name='bug2_nav_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add commands to launch description
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(bug2_nav_node)

    return ld