# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    model = DeclareLaunchArgument('model')
    side = DeclareLaunchArgument('side', default_value='')
    suffix = DeclareLaunchArgument(
            'suffix',
            default_value=PythonExpression(
                            '"" if side == "" else "_" + side,'
                        )
            )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='gripper' + suffix + '_controllers_spawner',
        output='screen',
        arguments=['--timeout', '120', 'gripper' + suffix + '_controller']
    )

    grasping_service_launch = IncludeLaunchDescription(
        launch_description_source=LaunchDescription([
            DeclareLaunchArgument('model'),
            DeclareLaunchArgument('side', default_value=''),
            Node(
                package='pal_robotiq_gripper_wrapper',
                executable='grasping_service',
                name='grasping_service',
                parameters=[{'model': model}, {'side': side}]
            )
        ])
    )

    ld = LaunchDescription()
    ld.add_action(model)
    ld.add_action(side)
    ld.add_action(suffix)
    ld.add_action(gripper_controller_spawner)
    ld.add_action(grasping_service_launch)

    return ld