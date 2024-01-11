# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

from launch import LaunchDescription
from launch_pal.include_utils import include_launch_py_description
from launch_ros.actions import Node


def generate_launch_description():
    robot_state_publisher = include_launch_py_description(
            'pal_robotiq_description', ['launch', 'robot_state_publisher_85.launch.py'])

    start_joint_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen')

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        #       arguments=['-d', rviz_config_file],
        output='screen')

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher)
    ld.add_action(start_joint_pub_gui)
    ld.add_action(start_rviz_cmd)

    return ld
