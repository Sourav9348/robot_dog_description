from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('robot_dog_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'Assem1_urdf_copy.urdf')

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters={'robot_description': open(urdf_path).read()}
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_path, 'config', 'collision_only.rviz')]
        )
    ])
