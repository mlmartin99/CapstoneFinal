import os, sys

from ament_index_python import get_package_share_directory


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    path_plan_node = Node(
        package="cpp_path_plan",
        executable="path_planner",
        parameters=['/home/G03/workspace/config/path_plan_params.yaml']
    )

    detection = Node(
        package="camera_distributor",
        executable="distributor"
    )

    uart_bridge = Node(
        package="uart-bridge",
        executable="uart_bridge"
    )
    pilot = Node(
        package="pilot",
        executable="pilot",
        parameters=['/home/G03/workspace/config/manual_params.yaml']
    )

    state_machine = Node(
        package="state_machine",
        executable="state_machine"
    )

    ui_translation = Node(
        package="ui_translation",
        executable="ui_translation"
    )

    # ros2 run web_video_server web_video_server
    web_video_server = Node(
            package='web_video_server',
            executable='web_video_server',
        )

    # include xml launch file
    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch/rosbridge_websocket_launch.xml",
            )
        )
    )

    ld.add_action(path_plan_node)
    ld.add_action(detection)
    ld.add_action(uart_bridge)
    ld.add_action(pilot)
    ld.add_action(state_machine)
    ld.add_action(ui_translation)
    ld.add_action(web_video_server)
    ld.add_action(rosbridge)

    return ld