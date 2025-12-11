source install/setup.bash


# 全部joint指定目标位置，并且需要在2秒后到达
ros2 topic pub --once /arm_group_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2', 'joint3'],
  points: [{
    positions: [1.0, 0.5, -0.5],
    velocities: [0.0, 0.0, 0.0],
    time_from_start: {sec: 2, nanosec: 0}  # 轨迹总共执行的时间
  }]
}"

sleep 10
echo "================== 连续动作 ==========================="

# 也可以指定某一部分joint
ros2 topic pub --once /arm_group_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2'],
  points: [{
    positions: [1.0, 0.5],
    velocities: [0.0, 0.0],
    time_from_start: {sec: 2, nanosec: 0}  # 第2秒到达
    },{
    positions: [0.0, 0.0],
    velocities: [0.0, 0.0],
    time_from_start: {sec: 4, nanosec: 0}  # 第4秒结束
  }]
}"