cmake_minimum_required(VERSION 3.8)
project(my_robot_urdf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 输出 clangd需要的 compile_commands.json

# 查找依赖包（必须包含所有C++代码用到的库）
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # 处理Python部分
find_package(hardware_interface REQUIRED)  # ros2_control硬件接口
find_package(pluginlib REQUIRED)           # 插件注册
find_package(rclcpp REQUIRED)              # C++节点库

# --------------------------
# 编译C++硬件接口插件
# --------------------------
add_library(
  my_hardware_controller  # 库名称
  SHARED           # 动态库（必须为SHARED，插件需要动态加载）
  src/my_hardware_controller.cpp  # 源代码路径
)

# 设置库的头文件路径
target_include_directories(
  my_hardware_controller
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>  # 编译时头文件
  $<INSTALL_INTERFACE:include>  # 安装后头文件路径（可选，这里简化）
)

# 链接依赖库（将硬件接口与依赖库关联）
ament_target_dependencies(
  my_hardware_controller
  hardware_interface
  pluginlib
  rclcpp
)

# 注册插件（关键步骤：让系统知道这是一个ros2_control硬件插件）
pluginlib_export_plugin_description_file(
  hardware_interface  # 插件所属的基类包
  my_hardware_controller.xml  # 插件描述文件（需与实际文件名一致）
)

# 安装C++库到系统lib目录
install(
  TARGETS my_hardware_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib  # 动态库安装路径（必须是系统能找到的lib目录）
  RUNTIME DESTINATION bin
)

# 安装插件描述文件（ros2_control需要读取该文件找到插件）
install(
  FILES my_hardware_controller.xml
  DESTINATION share/${PROJECT_NAME}  # 安装到包的share目录
)

# add_executable(my_node src/my_node.cpp)
# target_include_directories(my_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS my_node
#   DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config urdf launch rviz
  DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
