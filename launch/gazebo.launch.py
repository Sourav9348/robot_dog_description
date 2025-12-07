from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('robot_dog_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'new_assembly_URDF.urdf')

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot_dog',
            '-file', urdf_file,   
            '-x', '0', '-y', '0', '-z', '0.1',
            '-R', '0', '-P', '0', '-Y', '0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot
    ])
