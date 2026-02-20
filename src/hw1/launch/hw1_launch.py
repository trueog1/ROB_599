#!/usr/bin/env python3

import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='hw1',
            executable='hw1_code',
            name='talker'),
    ])