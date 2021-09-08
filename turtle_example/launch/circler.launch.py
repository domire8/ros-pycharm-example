from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
        output="screen",
    )

    turtle_example_node = Node(
        package='turtle_example',
        executable='circler',
        name='turtle_example',
    )

    return LaunchDescription([
        turtlesim_node,
        turtle_example_node,
    ])
