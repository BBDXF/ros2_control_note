
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
        control_level_.resize(info_.joints.size(), integration_level_t::POSITION);

        RCLCPP_WARN(get_logger(), "on_init, info_.joints.size() = %d", info_.joints.size());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            RCLCPP_WARN(get_logger(), "export_state_interfaces, export state: %d, %s", i, info_.joints[i].name.c_str());
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            RCLCPP_WARN(get_logger(), "export_command_interfaces, export command: %d, %s", i, info_.joints[i].name.c_str());
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override
    {
        RCLCPP_WARN(get_logger(), "prepare_command_mode_switch");
        // Prepare for new command modes
        std::vector<integration_level_t> new_modes = {};
        for (std::string key : start_interfaces)
        {
            RCLCPP_WARN(get_logger(), "prepare_command_mode_switch, prepare command: %s", key.c_str());
            for (std::size_t i = 0; i < info_.joints.size(); i++)
            {
                if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
                {
                    new_modes.push_back(integration_level_t::POSITION);
                }
                if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
                {
                    new_modes.push_back(integration_level_t::VELOCITY);
                }
                if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION)
                {
                    new_modes.push_back(integration_level_t::ACCELERATION);
                }
            }
        }
        
        // Example criteria: All joints must be given new command mode at the same time
        if (new_modes.size() != info_.joints.size())
        {
            return hardware_interface::return_type::ERROR;
        }
        // Example criteria: All joints must have the same command mode
        if (!std::all_of(new_modes.begin() + 1, new_modes.end(), [&](integration_level_t mode) { return mode == new_modes[0]; }))
        {
            return hardware_interface::return_type::ERROR;
        }

        // Stop motion on all relevant joints that are stopping
        for (std::string key : stop_interfaces)
        {
            for (std::size_t i = 0; i < info_.joints.size(); i++)
            {
                if (key.find(info_.joints[i].name) != std::string::npos)
                {
                    hw_commands_velocities_[i] = 0;
                    control_level_[i] = integration_level_t::UNDEFINED; // Revert to undefined
                }
            }
        }
        // Set the new command modes
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            if (control_level_[i] != integration_level_t::UNDEFINED)
            {
                // Something else is using the joint! Abort!
                return hardware_interface::return_type::ERROR;
            }
            control_level_[i] = new_modes[i];
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override
    {
        RCLCPP_WARN(get_logger(), "on_activate ...");

        // Set some default values
        for (std::size_t i = 0; i < hw_states_positions_.size(); i++)
        {
            hw_states_positions_[i] = 0;
            hw_states_velocities_[i] = 0;
        }
        for (std::size_t i = 0; i < hw_commands_positions_.size(); i++)
        {
            hw_commands_positions_[i] = 0;
            hw_commands_velocities_[i] = 0;
            control_level_[i] = integration_level_t::UNDEFINED;
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
        for (std::size_t i = 0; i < hw_states_positions_.size(); i++)
        {
            //     switch (control_level_[i])
            //     {
            //     case integration_level_t::UNDEFINED:
            //         break;
            //     case integration_level_t::POSITION:
            //         hw_states_velocities_[i] = 0;
            //         hw_states_positions_[i] += (hw_commands_positions_[i] - hw_states_positions_[i]) / hw_slowdown_;
            //         break;
            //     case integration_level_t::VELOCITY:
            //         hw_states_positions_[i] += (hw_states_velocities_[i] * period.seconds()) / hw_slowdown_;
            //         break;
            //     default:
            //         break;

            RCLCPP_WARN(get_logger(), "read. [%d] control_level: %d, pos state: %.2f, pos command: %.2f...", i, control_level_[i], hw_states_positions_[i],
                         hw_commands_positions_[i]);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override
    {
        for (std::size_t i = 0; i < hw_commands_positions_.size(); i++)
        {
            RCLCPP_WARN(get_logger(), "write. [%d] control_level: %d, pos state: %.2f, pos command: %.2f...", i, control_level_[i], hw_states_positions_[i],
                         hw_commands_positions_[i]);
            hw_states_positions_[i] = hw_commands_positions_[i];
            hw_states_velocities_[i] = (hw_commands_positions_[i] - hw_states_positions_[i])/10;
        }
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

    // Enum defining at which control level we are
    // Dumb way of maintaining the command_interface type per joint.
    enum integration_level_t : std::uint8_t
    {
        UNDEFINED = 0,
        POSITION = 1,
        VELOCITY = 2,
        ACCELERATION = 3
    };

    // Active control mode for each actuator
    std::vector<integration_level_t> control_level_;
};
} // namespace my_robot_urdf

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_urdf::MyHardwareController, hardware_interface::SystemInterface)
