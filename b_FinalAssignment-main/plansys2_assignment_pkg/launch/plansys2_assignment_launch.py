# Copyright 2019 Intelligent Robotics Lab
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_assignment_pkg')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/ass_example.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions    
    bug_node = Node(
        package='plansys2_assignment_pkg',  # Replace with the actual package name
        executable='bug0.py',
        name='bug0',
    )
    
    go_srv_node = Node(
        package='plansys2_assignment_pkg',  # Replace with the actual package name
        executable='go_to_point_srv.py',
        name='go_to_point_srv',
    )
    
    wall_srv_node = Node(
        package='plansys2_assignment_pkg',  # Replace with the actual package name
        executable='wall_follow_srv.py',
        name='wall_follow_srv',
    )

    go_to_action_node = Node(
        package='plansys2_assignment_pkg',  # Replace with the actual package name
        executable='go_to_action_node.py',
        name='go_to_action_node',
    )
    
    motor_motion_node = Node(
        package='plansys2_assignment_pkg',  # Replace with the actual package name
        executable='motor_motion_node.py',
        name='motor_motion_node',
    )
    
    camera_check_node = Node(
        package='plansys2_assignment_pkg',  # Replace with the actual package name
        executable='camera_check_node.py',
        name='camera_check_node',
    )
    
    marker_searcher_action = Node(
        package='plansys2_assignment_pkg',  # Replace with the actual package name
        executable='marker_searcher_action.py',
        name='marker_searcher_action',
    )
    
    aruco_generate_marker_node = Node(
        package='ros2_aruco',
        executable='aruco_generate_marker',
        name='aruco_generate_marker',
    )
    
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node',
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(bug_node)
    ld.add_action(go_srv_node)
    ld.add_action(wall_srv_node)
    ld.add_action(go_to_action_node)
    ld.add_action(motor_motion_node)
    ld.add_action(camera_check_node)
    ld.add_action(marker_searcher_action)
    ld.add_action(aruco_generate_marker_node)
    ld.add_action(aruco_node)

    return ld
