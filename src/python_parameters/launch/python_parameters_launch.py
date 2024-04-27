from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('python_parameters'),
        'config',
        'params.yaml'
        )

    return LaunchDescription([
        Node(
            package='python_parameters',
            executable='callback_param_node',
            name='callback_param_node',
            output='screen',
            emulate_tty=True,
            parameters=[config],
            # parameters=[
            #     {'my_parameter': 'earth'},
            # ]
        )
    ])