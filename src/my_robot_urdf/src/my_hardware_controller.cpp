// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_robot_urdf
{

class MyHardwareController : public hardware_interface::SystemInterface
{
  public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override
    {
        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(logger_, "Configuring ...please wait...");

        // 配置时重置所有值
        for (const auto &[name, descr] : joint_state_interfaces_)
        {
            set_state(name, 0.0);
        }
        for (const auto &[name, descr] : joint_command_interfaces_)
        {
            set_command(name, 0.0);
        }
        for (const auto &[name, descr] : gpio_state_interfaces_)
        {
            set_state(name, 0.0);
        }
        for (const auto &[name, descr] : gpio_command_interfaces_)
        {
            set_command(name, 0.0);
        }

        RCLCPP_INFO(logger_, "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(logger_, "Activating ...please wait...");

        // 启动时命令和状态应相等
        // for (const auto &[name, descr] : joint_state_interfaces_)
        // {
        //     set_command(name, get_state(name));
        // }
        // for (const auto &[name, descr] : gpio_command_interfaces_)
        // {
        //     set_command(name, get_state(name));
        // }

        RCLCPP_INFO(logger_, "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(logger_, "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    // {
    //     std::vector<hardware_interface::StateInterface> state_interfaces;

    //     for (const auto &joint : info_.joints)
    //     {
    //         for (const auto &state_interface : joint.state_interfaces)
    //         {
    //             state_interfaces.emplace_back(
    //                 hardware_interface::StateInterface(joint.name, state_interface.name, &joint_state_interfaces_[joint.name + "/" + state_interface.name]));
    //         }
    //     }

    //     for (const auto &gpio : info_.gpios)
    //     {
    //         for (const auto &state_interface : gpio.state_interfaces)
    //         {
    //             state_interfaces.emplace_back(hardware_interface::StateInterface(gpio.name, state_interface.name, &gpio_state_interfaces_[gpio.name + "/" + state_interface.name]));
    //         }
    //     }

    //     return state_interfaces;
    // }

    // std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    // {
    //     std::vector<hardware_interface::CommandInterface> command_interfaces;

    //     for (const auto &joint : info_.joints)
    //     {
    //         for (const auto &command_interface : joint.command_interfaces)
    //         {
    //             command_interfaces.emplace_back(
    //                 hardware_interface::CommandInterface(joint.name, command_interface.name, &joint_command_interfaces_[joint.name + "/" + command_interface.name]));
    //         }
    //     }

    //     for (const auto &gpio : info_.gpios)
    //     {
    //         for (const auto &command_interface : gpio.command_interfaces)
    //         {
    //             command_interfaces.emplace_back(
    //                 hardware_interface::CommandInterface(gpio.name, command_interface.name, &gpio_command_interfaces_[gpio.name + "/" + command_interface.name]));
    //         }
    //     }

    //     return command_interfaces;
    // }

    hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period) override
    {
        std::stringstream ss;
        ss << "Reading states:";

        // 模拟关节运动
        for (const auto &joint : info_.joints)
        {
            const std::string pos_state_name = joint.name + "/" + hardware_interface::HW_IF_POSITION;
            const std::string vel_state_name = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
            const std::string pos_cmd_name = joint.name + "/" + hardware_interface::HW_IF_POSITION;

            // 更新位置状态
            double new_pos = get_state(pos_state_name) + (get_command(pos_cmd_name) - get_state(pos_state_name)) / hw_slowdown_;
            set_state(pos_state_name, new_pos);

            // 计算并更新速度状态
            double velocity = (get_command(pos_cmd_name) - get_state(pos_state_name)) / period.seconds();
            set_state(vel_state_name, velocity);

            ss << std::fixed << std::setprecision(2) << std::endl << "\t" << get_state(pos_state_name) << " for joint '" << joint.name << "'";
        }

        // 模拟GPIO输入
        // 模拟数字输入
        unsigned int seed = static_cast<unsigned int>(time(NULL)) + 1;
        const std::string digital_input_name = info_.gpios[0].name + "/" + info_.gpios[0].state_interfaces[0].name;
        set_state(digital_input_name, static_cast<double>(rand_r(&seed) % 2));

        // 模拟模拟输入
        seed = static_cast<unsigned int>(time(NULL)) + 2;
        const std::string analog_input_name = info_.gpios[0].name + "/" + info_.gpios[0].state_interfaces[1].name;
        set_state(analog_input_name, static_cast<double>(rand_r(&seed) % 1000) / 1000.0);

        // 回读GPIO输出作为输入
        for (size_t i = 0; i < info_.gpios[1].state_interfaces.size(); i++)
        {
            const std::string state_name = info_.gpios[1].name + "/" + info_.gpios[1].state_interfaces[i].name;
            const std::string cmd_name = info_.gpios[1].name + "/" + info_.gpios[1].command_interfaces[i].name;
            set_state(state_name, get_command(cmd_name));

            ss << std::fixed << std::setprecision(2) << std::endl << "\t" << get_state(state_name) << " from GPIO input '" << state_name << "'";
        }

        RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 500, ss.str());

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
    {
        std::stringstream ss;
        ss << "Writing commands:";

        // 模拟发送命令到硬件
        for (const auto &[name, descr] : gpio_command_interfaces_)
        {
            // 模拟发送命令到硬件
            ss << std::fixed << std::setprecision(2) << std::endl << "\t" << get_command(name) << " for GPIO output '" << name << "'";
        }

        for (const auto &[name, descr] : joint_command_interfaces_)
        {
            ss << std::fixed << std::setprecision(2) << std::endl << "\t" << get_command(name) << " for joint '" << name.substr(0, name.find("/")) << "'";
        }

        RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 500, ss.str());

        return hardware_interface::return_type::OK;
    }

  private:
    // 硬件减速因子
    double hw_slowdown_ = 50.0; // 模拟硬件响应延迟
    rclcpp::Logger logger_ = rclcpp::get_logger("MyHardwareController");
};

} // namespace my_robot_urdf

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_urdf::MyHardwareController, hardware_interface::SystemInterface)