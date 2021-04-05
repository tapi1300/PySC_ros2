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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('house_navigation')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/house_domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    move_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_1',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])
    move_2_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_2',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])
    move_3_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_3',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])
    
    move_to_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_to_zone',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move_to_zone',
            'publisher_port': 1682,
            'server_port': 1683,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])
    move_out_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_out_zone',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move_out_zone',
            'publisher_port': 1684,
            'server_port': 1685,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])


    pick_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pick',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'pick',
            'publisher_port': 1686,
            'server_port': 1687,
            'bt_xml_file': example_dir + '/behavior_trees_xml/pick.xml'
          }
        ])

    drop_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='drop',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'drop',
            'publisher_port': 1688,
            'server_port': 1689,
            'bt_xml_file': example_dir + '/behavior_trees_xml/drop.xml'
          }
        ])


      

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_1_cmd)
    ld.add_action(move_2_cmd)
    ld.add_action(move_3_cmd)
    ld.add_action(move_to_cmd)
    ld.add_action(move_out_cmd)
    ld.add_action(pick_cmd)
    ld.add_action(drop_cmd)

    return ld
