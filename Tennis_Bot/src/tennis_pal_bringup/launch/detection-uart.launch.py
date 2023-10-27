

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


    detection = Node(
        package="camera_distributor",
        executable="distributor"
    )

    uart_bridge = Node(
        package="uart-bridge",
        executable="uart_bridge"
    )

    ld.add_action(detection)
    ld.add_action(uart_bridge)

    return ld