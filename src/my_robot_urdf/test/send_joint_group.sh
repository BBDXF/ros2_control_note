source install/setup.bash

# joint1, joint2, joint3
ros2 topic pub --once /arm_group_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.3, 0.8]"

