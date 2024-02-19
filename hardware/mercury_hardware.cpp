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

#include "tr_robot/mercury_hardware.hpp"
#include <string>
#include <vector>

namespace tr_robot
{
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
 
  // robot has 6 joints and therefore we inistialize for every key a vector with 6 values
  const int vectorSize = 6; // Size of the vector

  // Initialize vectors with default values
  for (auto& entry : joint_command_interfaces) {
    entry.second.assign(vectorSize, 0.0);
  }

  for (auto& entry : joint_interfaces) {
    entry.second.assign(vectorSize, 0.0);
  }

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["acceleration"])
  {
    state_interfaces.emplace_back(joint_name, "acceleration", &joint_velocities_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_command_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_command_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_command_interfaces["acceleration"])
  {
    command_interfaces.emplace_back(joint_name, "acceleration", &joint_velocities_command_[ind++]);
  }

  return command_interfaces;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    for (int i = 0; i < 6; i++) {
      EncoderEstimates estimates = Hndl.GetEncoderEstimate(i);

      punning_position.u = estimates.Position;
      punning_velocity.u = estimates.Velocity;

      joint_interfaces["position"][i] = punning_position.f / 100;
      joint_interfaces["velocity"][i] = punning_velocity.f / 100;
    }

  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (int i = 0; i < 6; i++) {
    double punning_position.f = joint_command_interfaces["position"][i] * 100;
    double punning_velocity.f = joint_command_interfaces["velocity"][i] * 100;
    double punning_acceleration.f = joint_command_interfaces["acceleration"][i] * 100;

    Hndl.SetInputPos(i, punning_position.u, punning_velocity.u, punning_acceleration.u);
  }
  return return_type::OK;
}

}  // namespace tr_robot

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tr_robot::RobotSystem, hardware_interface::SystemInterface)
