// Copyright 2021 ros2_control Development Team
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

#ifndef REAL_CAR__CARLIKEBOT_SYSTEM_HPP_
#define REAL_CAR__CARLIKEBOT_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "real_car/visibility_control.h"

namespace real_car
{

// This is the node definition for the publisher to talk to microROS agent
class HardwareCommandPub : public rclcpp::Node
{
  public:
    HardwareCommandPub();
    void publishSpeed(int speed);
    void publishAngle(double angle);
  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

};

struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  explicit Joint(const std::string & name) : joint_name(name)
  {
    state = JointValue();
    command = JointValue();
  }

  Joint() = default;

  std::string joint_name;
  JointValue state;
  JointValue command;
};

class RealCarHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RealCarHardware);

  REAL_CAR_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  REAL_CAR_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  REAL_CAR_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  REAL_CAR_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  REAL_CAR_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  REAL_CAR_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  REAL_CAR_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::shared_ptr<HardwareCommandPub> hw_cmd_pub_;    //make the publisher node a member

  // function defintion to convert normalized twist.linear.x to pwm
  void velToPWM(double vel, int& motorPWM);

private:

  // std::vector<std::tuple<std::string, double, double>>
  //   hw_interfaces_;  // name of joint, state, command
  std::map<std::string, Joint> hw_interfaces_;
  int motorPWM;
  int servoPWM;  
  double normalizedSpeed;
  double normalizedAngle;  
  double speed2publish;
  double angle2publish;
};

}  // namespace real_car

#endif  // REAL_CAR__CARLIKEBOT_SYSTEM_HPP_
