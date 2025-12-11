# ros2_control_note
The note used to learn URDF and ros2_control system.

Note:
- ROS humble 与 Humble 在ros2_control URDF 和 interface上有很多不同，不可以兼容。
- Ubuntu 22.04 + Humble 下开发测试.
- 请使用github上最新版本查看。

## 系统环境
- Ubuntu 22.04
- ROS2 humble

参考URL:
- https://wiki.ros.org/urdf/XML
- https://control.ros.org/master/doc/getting_started/getting_started.html#architecture
- https://control.ros.org/master/doc/ros2_control_demos/example_10/doc/userdoc.html
- https://github.com/ros-controls/ros2_control_demos


## 依赖库
```bash
sudo apt install mesa-utils build-essential cmake git curl tree clangd gcc
sudo apt install ros-humble-desktop ros-dev-tools
sudo apt install ros-humble-ros2-controllers ros-humble-ros2-control ros-humble-joint-trajectory-controller
sudo apt install ros-humble-joint-state-publisher-gui ros-humble-xacro
```

查看支持的 controller_types:
```bash
# 需要首先启动一个control_manager


$ ros2 control list_controller_types
effort_controllers/GripperActionController                             controller_interface::ControllerInterface
effort_controllers/JointGroupEffortController                          controller_interface::ControllerInterface
forward_command_controller/ForwardCommandController                    controller_interface::ControllerInterface
forward_command_controller/MultiInterfaceForwardCommandController      controller_interface::ControllerInterface
gpio_controllers/GpioCommandController                                 controller_interface::ControllerInterface
gps_sensor_broadcaster/GPSSensorBroadcaster                            controller_interface::ControllerInterface
imu_sensor_broadcaster/IMUSensorBroadcaster                            controller_interface::ControllerInterface
joint_state_broadcaster/JointStateBroadcaster                          controller_interface::ControllerInterface
joint_trajectory_controller/JointTrajectoryController                  controller_interface::ControllerInterface
parallel_gripper_action_controller/GripperActionController             controller_interface::ControllerInterface
pose_broadcaster/PoseBroadcaster                                       controller_interface::ControllerInterface
position_controllers/GripperActionController                           controller_interface::ControllerInterface
position_controllers/JointGroupPositionController                      controller_interface::ControllerInterface
range_sensor_broadcaster/RangeSensorBroadcaster                        controller_interface::ControllerInterface
tricycle_controller/TricycleController                                 controller_interface::ControllerInterface
velocity_controllers/JointGroupVelocityController                      controller_interface::ControllerInterface
ackermann_steering_controller/AckermannSteeringController              controller_interface::ChainableControllerInterface
admittance_controller/AdmittanceController                             controller_interface::ChainableControllerInterface
bicycle_steering_controller/BicycleSteeringController                  controller_interface::ChainableControllerInterface
chained_filter_controller/ChainedFilter                                controller_interface::ChainableControllerInterface
diff_drive_controller/DiffDriveController                              controller_interface::ChainableControllerInterface
force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster           controller_interface::ChainableControllerInterface
mecanum_drive_controller/MecanumDriveController                        controller_interface::ChainableControllerInterface
omni_wheel_drive_controller/OmniWheelDriveController                   controller_interface::ChainableControllerInterface
pid_controller/PidController                                           controller_interface::ChainableControllerInterface
tricycle_steering_controller/TricycleSteeringController                controller_interface::ChainableControllerInterface
```
## Steps

### 1. 创建包和URDF

### 2. 添加 ros2_control

### 3. launch文件与ros2_control yaml配置

### 4. 编写 hardware 插件

### 5. CMakeLists.txt

### 6. 总结


####################

joint\_states 规划: `position_controllers/JointGroupPositionController`:

```bash
$ ros2 node list
/arm_group_controller
/controller_manager
/joint_state_broadcaster
/robot_state_publisher
/rviz2
/transform_listener_impl_6135dbfae7f0

$ ros2 control list_controllers 
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster      active
arm_group_controller    position_controllers/JointGroupPositionController  active

$ ros2 control list_hardware_interfaces 
command interfaces
	joint1/position [available] [claimed]
	joint1/velocity [available] [unclaimed]
	joint2/position [available] [claimed]
	joint2/velocity [available] [unclaimed]
	joint3/position [available] [claimed]
	joint3/velocity [available] [unclaimed]
state interfaces
	joint1/position
	joint1/velocity
	joint2/position
	joint2/velocity
	joint3/position
	joint3/velocity
	
$ ros2 topic list 
/arm_group_controller/commands
/arm_group_controller/transition_event
/clicked_point
/dynamic_joint_states
/goal_pose
/initialpose
/joint_state_broadcaster/transition_event
/joint_states
/parameter_events
/robot_description
/rosout
/tf
/tf_static

$ ros2 topic echo /joint_states --once
header:
  stamp:
    sec: 1765193813
    nanosec: 460238490
  frame_id: base_link
name:
- joint1
- joint2
- joint3
position:
- 0.0
- 0.0
- 0.0
velocity:
- 0.0
- 0.0
- 0.0
effort:
- .nan
- .nan
- .nan
---

$ ros2 topic pub --once /arm_group_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.3, 0.8]"
publisher: beginning loop
publishing #1: std_msgs.msg.Float64MultiArray(layout=std_msgs.msg.MultiArrayLayout(dim=[], data_offset=0), data=[0.5, -0.3, 0.8])

```

轨迹规划：`joint_trajectory_controller/JointTrajectoryController`

```bash

```


