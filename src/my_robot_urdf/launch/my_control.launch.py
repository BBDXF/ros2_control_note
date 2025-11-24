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
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            PathJoinSubstitution([FindPackageShare("my_robot_urdf"), "config", "my_control_conf.yaml"])
        ],
    )

    joint_state_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_controller", "--controller-manager", "/controller_manager"],
    )

    # 3. 关节组控制器（支持单/多关节控制）
    arm_group_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_group_controller", "--controller-manager", "/controller_manager"],
    )

    # 4. 夹爪控制器
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # 5. RViz节点
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("my_robot_urdf"), "rviz", "view_arm.rviz"])],
    )

    # 6. 退出事件（停止所有控制器）
    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[
                Node(package="controller_manager", executable="unspawner", arguments=["arm_group_controller", "--controller-manager", "/controller_manager"]),
                Node(package="controller_manager", executable="unspawner", arguments=["gripper_controller", "--controller-manager", "/controller_manager"]),
                Node(package="controller_manager", executable="unspawner", arguments=["joint_state_controller", "--controller-manager", "/controller_manager"]),
            ],
        )
    )

    # 7. 启动所有节点
    return LaunchDescription([
        controller_manager_node,
        robot_state_publisher_node,
        joint_state_controller_spawner,
        arm_group_controller_spawner,
        gripper_controller_spawner,
        # joint_state_publisher_node,
        joint_state_broadcaster_spawner,
        rviz_node,
        exit_event_handler
    ])
