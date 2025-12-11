
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

namespace my_robot_urdf
{

class MyHardwareController : public hardware_interface::SystemInterface
{
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MyHardwareController)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("MyHardwareController"));
        clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

        // info_ 是 info 的内部存储变量. 通过 SystemInterface::on_init 赋值
        hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        // gpio size = gpio node count * interface count
        gpio_commands_positions_.resize(info_.gpios.size()*1, std::numeric_limits<double>::quiet_NaN());
        gpio_states_positions_.resize(info_.gpios.size()*1, std::numeric_limits<double>::quiet_NaN());

        RCLCPP_WARN(get_logger(), "on_init, joints size: %d, gpio size: %d", info_.joints.size(), info_.gpios.size());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            RCLCPP_WARN(get_logger(), "export_state_interfaces, joint state: %d, %s", i, info_.joints[i].name.c_str());
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
        }

        size_t ct = 0;
        for (size_t i = 0; i < info_.gpios.size(); i++)
        {
            for (auto state_if : info_.gpios.at(i).state_interfaces)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(info_.gpios.at(i).name, state_if.name, &gpio_states_positions_[ct++]));
                RCLCPP_INFO(get_logger(), "export_state_interfaces, gpio state: %s/%s", info_.gpios.at(i).name.c_str(), state_if.name.c_str());
            }
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            RCLCPP_WARN(get_logger(), "export_command_interfaces, joint command: %d, %s", i, info_.joints[i].name.c_str());
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
        }

        size_t ct = 0;
        for (size_t i = 0; i < info_.gpios.size(); i++)
        {
            for (auto command_if : info_.gpios.at(i).command_interfaces)
            {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.gpios.at(i).name, command_if.name, &gpio_commands_positions_[ct++]));
                RCLCPP_INFO(get_logger(), "export_command_interfaces, gpio command: %s/%s", info_.gpios.at(i).name.c_str(), command_if.name.c_str());
            }
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override
    {
        RCLCPP_WARN(get_logger(), "on_activate ...");

        // Set default values
        for (std::size_t i = 0; i < hw_states_positions_.size(); i++)
        {
            hw_states_positions_[i] = 0;
            hw_states_velocities_[i] = 0;
        }
        for (std::size_t i = 0; i < hw_commands_positions_.size(); i++)
        {
            hw_commands_positions_[i] = 0;
            hw_commands_velocities_[i] = 0;
        }

        for (std::size_t i = 0; i < gpio_states_positions_.size(); i++)
        {
            gpio_states_positions_[i] = 0;
        }
        for (std::size_t i = 0; i < gpio_commands_positions_.size(); i++)
        {
            gpio_commands_positions_[i] = 0;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override
    {
        RCLCPP_WARN(get_logger(), "on_deactivate ...");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override
    {
        static int count = 0;
        if (count++ % 200 != 0)
        {
            return hardware_interface::return_type::OK;
        }
        std::ostringstream ss;
        ss << "joints: [ " << std::fixed << std::setprecision(3);
        for (std::size_t i = 0; i < hw_states_positions_.size(); i++)
        {
            ss << hw_states_positions_[i] << ", ";
        }
        ss << "] ";
        ss << "gpios: [ ";
        for (std::size_t i = 0; i < gpio_states_positions_.size(); i++)
        {
            ss << gpio_states_positions_[i] << ", ";
        }
        ss << "] ";
        RCLCPP_WARN(get_logger(), "read: %s .", ss.str().c_str());
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override
    {
        bool changed = false;
        std::ostringstream ss;
        ss << "[ " << std::fixed << std::setprecision(3);
        for (std::size_t i = 0; i < hw_commands_positions_.size(); i++)
        {
            if (abs(hw_commands_positions_[i] - hw_states_positions_[i]) > 0.001){
                changed = true;
            }else{
                // 当前joint 状态和目标不一致, 需要发送控制指令 
            }
            hw_states_positions_[i] = hw_commands_positions_[i];
            hw_states_velocities_[i] = (hw_commands_positions_[i] - hw_states_positions_[i]) / 10;

            ss << hw_commands_positions_[i] << ", ";
        }
        ss << "] ";
        ss << "gpios: [ ";
        for (std::size_t i = 0; i < gpio_commands_positions_.size(); i++)
        {
            if (abs(gpio_commands_positions_[i] - gpio_states_positions_[i]) > 0.001){
                changed = true;
            }else{
                // 当前gpio 状态和目标不一致, 需要发送控制指令 
            }
            ss << gpio_commands_positions_[i] << ", ";
            gpio_states_positions_[i] = gpio_commands_positions_[i];
        }
        ss << "] ";
        if(changed)
            RCLCPP_WARN(get_logger(), "write (%.2f) ==> %s .", period.seconds(), ss.str().c_str());
        return hardware_interface::return_type::OK;
    }

    /// Get the logger of the SystemInterface.
    /**
     * \return logger of the SystemInterface.
     */
    rclcpp::Logger get_logger() const
    {
        return *logger_;
    }

    /// Get the clock of the SystemInterface.
    /**
     * \return clock of the SystemInterface.
     */
    rclcpp::Clock::SharedPtr get_clock() const
    {
        return clock_;
    }

  private:
    // Objects for logging
    std::shared_ptr<rclcpp::Logger> logger_;
    rclcpp::Clock::SharedPtr clock_;

    // Store the commands for the simulated robot
    std::vector<double> hw_commands_positions_;
    std::vector<double> hw_commands_velocities_;
    std::vector<double> hw_states_positions_;
    std::vector<double> hw_states_velocities_;
    std::vector<double> gpio_states_positions_;
    std::vector<double> gpio_commands_positions_;
};
} // namespace my_robot_urdf

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_urdf::MyHardwareController, hardware_interface::SystemInterface)
