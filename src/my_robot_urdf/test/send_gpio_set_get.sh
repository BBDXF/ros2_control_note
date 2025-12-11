source install/setup.bash

# list
ros2 control list_controllers
ros2 control list_hardware_interfaces 
ros2 topic list | grep gpio

# gpio set
# /gpio_controller/commands
ros2 topic pub --once /gpio_controller/commands control_msgs/msg/DynamicInterfaceGroupValues \
    "{interface_groups: [gpio_test], interface_values: [{interface_names: [emstop_input], values: [0.27]}]}"


# gpio get
# /gpio_controller/gpio_states
ros2 topic echo --once /gpio_controller/gpio_states

