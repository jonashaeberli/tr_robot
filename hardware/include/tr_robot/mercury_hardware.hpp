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

#ifndef TR_ROBOT__MERCURY_HARDWARE_HPP_
#define TR_ROBOT__MERCURY_HARDWARE_HPP_

#include "string"
#include "unordered_map"
#include "vector"

#include "../ODrive-CAN/ODrive.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::return_type;

namespace tr_robot
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:

  ODrive::ODrive Hndl;

/// Here we create two maps that store the values of the input command beeing sent to the controller and the values read from the controllers...

  std::unordered_map<std::string, std::vector<std::double>> joint_command_interfaces = {
    {"position", {}}, {"velocity", {}}, {"acceleration", {}}};

  std::unordered_map<std::string, std::vector<std::double>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}, {"acceleration", {}}};

  union
  {
    float f;
    uint32_t u;
  }punning_position;

  union
  {
    float f;
    uint32_t u;
  }punning_velocity;

  union
  {
    float f;
    uint32_t u;
  }punning_acceleration;

  typedef struct {
    uint32_t Position;
    uint32_t Velocity;
} EncoderEstimates;
  
};

}  // namespace tr_robot

#endif  // TR_ROBOT__MERCURY_HARDWARE_HPP_
