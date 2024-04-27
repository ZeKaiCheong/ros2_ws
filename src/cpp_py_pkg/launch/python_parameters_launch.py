from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cpp_py_pkg'),
        'config',
        'params.yaml'
        )

    return LaunchDescription([
        Node(
            package='cpp_py_pkg',
            executable='callback_param_node.py',
            name='callback_param_node',
            output='screen',
            emulate_tty=True,
            parameters=[config],
            # parameters=[
            #     {'my_parameter': 'earth'},
            # ]
        )
    ])