# ros2_control_note
The note used to learn URDF and ros2_control system.
# Note
## 系统环境
- Ubuntu 24.04
- ROS2 jazzy

## 依赖库
```bash
sudo apt install mesa-utils build-essential cmake git curl tree clangd gcc
sudo apt install ros-jazzy-desktop ros-dev-tools
sudo apt install ros-jazzy-ros2-controllers ros-jazzy-ros2-control 
sudo apt install ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro
```

查看支持的 controller_types:
```bash
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
参考URL:
- https://wiki.ros.org/urdf/XML
- https://control.ros.org/master/doc/getting_started/getting_started.html#architecture
- https://control.ros.org/master/doc/ros2_control_demos/example_10/doc/userdoc.html
- https://github.com/ros-controls/ros2_control_demos

### 1. 创建包和URDF

### 2. 添加 ros2_control

### 3. launch文件与ros2_control yaml配置

### 4. 编写 hardware 插件

### 5. CMakeLists.txt

### 6. 总结
