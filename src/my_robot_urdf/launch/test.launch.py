import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 加载URDF（修正后路径）
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("my_robot_urdf"), "urdf", "my_robot.urdf.xacro"]
    )
    robot_description = Command([FindExecutable(name="xacro"), " ", urdf_file])

    # 2. 基础节点（保持不变）
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    # )

    # 5. RViz节点
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("my_robot_urdf"), "rviz", "view_arm.rviz"])],
    )



    # 7. 启动所有节点
    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_node,
        rviz_node
    ])
