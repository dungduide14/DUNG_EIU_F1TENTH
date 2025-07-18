from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    Baitap_server = Node(
        package="ros_basic",
        executable="baitap_server.py",
        name="simple_service_server",
        output="screen",

    )
    Baitap_client = Node(
        package="ros_basic",
        executable="baitap_client.py",
        name="simple_service_client",
        output="screen",
        parameters=[
            {"distance_odom": 4.0},
            {"distance_lidar": 1.0}
        ],

    )
    return LaunchDescription([
        Baitap_server,
        Baitap_client
    ])