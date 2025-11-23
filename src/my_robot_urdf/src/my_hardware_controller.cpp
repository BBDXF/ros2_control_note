// my_hardware_controller.cpp
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>

namespace my_robot_urdf
{

class MyHardwareController : public hardware_interface::SystemInterface
{
  public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override
    {
        // 调用基类初始化
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        try
        {
            // 根据硬件类型初始化相应的接口
            if (info_.type == "actuator")
            {
                // 初始化执行器特定的数据结构
                RCLCPP_INFO(logger_, "Initializing actuator hardware component: %s", info_.name.c_str());
            }
            else if (info_.type == "sensor")
            {
                // 初始化传感器特定的数据结构
                RCLCPP_INFO(logger_, "Initializing sensor hardware component: %s", info_.name.c_str());
            }
            else if (info_.type == "system")
            {
                // 初始化系统特定的数据结构
                RCLCPP_INFO(logger_, "Initializing system hardware component: %s", info_.name.c_str());
            }

            // 初始化关节状态（位置、速度）
            joint_positions_.resize(info_.joints.size(), 0.0);
            joint_velocities_.resize(info_.joints.size(), 0.0);
            joint_commands_.resize(info_.joints.size(), 0.0);

            // 初始化GPIO状态
            gpio_states_.resize(info_.gpios.size(), 0.0);
            gpio_commands_.resize(info_.gpios.size(), 0.0);

            return hardware_interface::CallbackReturn::SUCCESS;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Failed to initialize hardware component: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            // 位置状态接口
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));

            // 速度状态接口
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
        }

        // 添加GPIO状态接口
        for (size_t i = 0; i < info_.gpios.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.gpios[i].name, "digital", &gpio_states_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_commands_[i]));
        }

        // 添加GPIO命令接口
        for (size_t i = 0; i < info_.gpios.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.gpios[i].name, "digital", &gpio_commands_[i]));
        }
        return command_interfaces;
    }

    // 修复：更改 read 方法的返回类型为 return_type
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override
    {
        // 从硬件读取数据并更新状态接口
        try
        {
            // 实现读取硬件状态的逻辑
            // 示例: 更新位置、速度等状态值
            for (size_t i = 0; i < joint_positions_.size(); i++)
            {
                // 简单模拟：位置向命令值移动（速度限制为0.5 rad/s）
                double error = joint_commands_[i] - joint_positions_[i];
                double max_step = 0.5 * period.seconds(); // 最大步长
                joint_positions_[i] += std::clamp(error, -max_step, max_step);

                // 计算速度（位置变化/时间）
                joint_velocities_[i] = error / period.seconds();
            }

            // 模拟GPIO输入状态更新
            for (size_t i = 0; i < gpio_states_.size(); i++)
            {
                gpio_states_[i] = gpio_commands_[i]; // 回读GPIO命令值作为状态
            }

            return hardware_interface::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Failed to read from hardware: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

    // 修复：更改 on_write 方法名称为 write 并修改返回类型为 return_type
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override
    {
        // 将命令接口的值写入硬件
        try
        {
            // 实现向硬件写入命令的逻辑
            // 示例: 发送位置、速度等命令到硬件
            // 在这个模拟中不需要额外操作，命令已在on_read中使用

            return hardware_interface::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Failed to write to hardware: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

  private:
    // 关节状态变量
    std::vector<double> joint_positions_;  // 关节当前位置
    std::vector<double> joint_velocities_; // 关节当前速度
    std::vector<double> joint_commands_;   // 关节目标位置命令
    // GPIO状态变量
    std::vector<double> gpio_states_;  // GPIO当前状态
    std::vector<double> gpio_commands_; // GPIO目标状态命令

    rclcpp::Logger logger_ = rclcpp::get_logger("MyHardwareController");
};

} // namespace my_robot_urdf

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_urdf::MyHardwareController, hardware_interface::SystemInterface)