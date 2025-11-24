from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('seda_controller')
    config = os.path.join(pkg_share, 'config', 'config.yaml')

    # === Dynamixel Node ===
    seda_dynamixel = Node(
        package='seda_controller',
        executable='seda_dynamixel_node',
        name='seda_dynamixel',
        output='screen',
        parameters=[config],
    )

    # === Controller Node ===
    seda_controller = Node(
        package='seda_controller',
        executable='seda_controller_node',
        name='seda_controller',
        output='screen',
        parameters=[config],
    )

    # === Encoder Bridge Node ===
    encoder_hex_node = Node(
        package='encoder_bridge',
        executable='encoder_angle_hex_node',
        name='encoder_bridge_hex',
        output='screen',
        parameters=[
            {'port': '/dev/ttyUSB0'},
            {'baudrate': 115200}
        ]
    )

    return LaunchDescription([
        encoder_hex_node,
        seda_dynamixel,
        seda_controller,
    ])

