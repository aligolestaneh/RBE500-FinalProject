import os
from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument)
import launch_ros.actions


def generate_launch_description():
    package_directory = get_package_share_directory('rbe500_final_project')
    default_params_file = Path(package_directory) / 'params' / 'manipulator_vlk_params.yaml'
     
    safety_node = launch_ros.actions.Node(
        package='rbe500_final_project',
        executable='manipulator_jp_updater_node',
        name='manipulator_jp_updater_node',
        # namespace='manipulator_core',
        # prefix=["gdb -ex run --args"],
        emulate_tty=True,  # set to True to enable colored logging
        output='screen',
        parameters=[default_params_file]
    )
    return launch.LaunchDescription([safety_node])

if __name__ == '__main__':
    generate_launch_description()