# Copyright (C) - All Rights Reserved
#
# Written by Alejandra Martínez Fariña <alejandra.mf23be@gmail.com>
# Licensed under the Apache License, Version 2.0

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define the topic name launch argument
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/camera/image_raw',
        description='Name of the image topic to subscribe to'
    )

    # Launch the C++ executable
    image_serializer_cpp_node = launch_ros.actions.Node(
        package='serializer_pkg',
        executable='image_serializer_cpp',
        output='screen',
        parameters=[{'topic_name': LaunchConfiguration('topic_name')}]
    )

    # Launch the Python executable
    image_serializer_python_node = launch_ros.actions.Node(
        package='serializer_pkg',
        executable='image_serializer_python.py',
        output='screen',
        emulate_tty=True,
        parameters=[{'topic_name': LaunchConfiguration('topic_name')}]
    )

    # Create the launch description
    ld = launch.LaunchDescription()

    # Add the launch arguments and nodes to the launch description
    ld.add_action(topic_name_arg)
    ld.add_action(image_serializer_cpp_node)
    ld.add_action(image_serializer_python_node)

    return ld

