// Created by Team 6 for EEC 195AB, Spring 2024
// Adrien Salvador, Raphael Bret-Mounet, Gabriel Castellanos

#include "real_car/real_car.hpp"
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
double feedbackSpeed;
double feedbackAngle;

namespace real_car
{
// Create the topic that the Pi will publish to and Pico will subscribe to
HardwareCommandPubMotor::HardwareCommandPubMotor() : Node("motor_publisher")
{
  motor_publisher_ = this->create_publisher<std_msgs::msg::String>("pi_motor_publishing_topic", 10);
    timer_ = this->create_wall_timer(
        70ms, std::bind(&HardwareCommandPubMotor::timer_callback, this));
}

// Create the topic that the Pi will publish to and Pico will subscribe to
HardwareCommandPubServo::HardwareCommandPubServo() : Node("servo_publisher")
{
  servo_publisher_ = this->create_publisher<std_msgs::msg::String>("pi_servo_publishing_topic", 10);
    timer_ = this->create_wall_timer(
            70ms, std::bind(&HardwareCommandPubMotor::timer_callback, this));
}

// Function for publishing to the topic that the Pico will subscribe to
void HardwareCommandPubMotor::publishSpeed(int speed, std::string direction)
{
  auto message = std_msgs::msg::String();
  message.data = std::to_string(speed) + " " + direction;
  motor_publisher_->publish(message);
}

// Function for publishing to the topic that the Pico will subscribe to
void HardwareCommandPubServo::publishAngle(int angle)
{
  auto message = std_msgs::msg::String();
  message.data = std::to_string(angle);
  servo_publisher_->publish(message);
}
//Create the subscriber
 HardwareCommandSubMotorServo::HardwareCommandSubMotorServo() : Node("motor_servo_subscriber")
{
    motor_subscriber_ = this->create_subscription<std_msgs::msg::String>(
       "control_topic", rclcpp::SensorDataQoS(),
       [this](const std_msgs::msg::String::SharedPtr msg) { servo_motor_callback(msg); });
}

 //Callback function for the subscriber
void HardwareCommandSubMotorServo::servo_motor_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    std::string msg_data = msg->data.c_str();

    //Message format: "angle speed"

    // Find the position of the space character
    size_t space_pos = msg_data.find(' ');
    if (space_pos == std::string::npos) {
        // Invalid message format, handle error or return
        return;
    }

    // Extract motor speed and servo angle
    feedbackAngle = std::stod(msg_data.substr(0, space_pos));
    feedbackSpeed = std::stod(msg_data.substr(space_pos + 1));
}

// Function for converting twist.linear.x to PWM signals
void RealCarHardware::motorVelToPWM(double vel, int& motorPWM, std::string& direction)
{
    // Define the mapping constants
    double maxSpeed = 1.0;     // Maximum speed
    int maxPWM = 2000000;       // Maximum PWM value (for max forward)
    int minPWM = 1375000;       // Minimum PWM value (for motor off)

    // Convert speed to PWM signal
    if (vel > 0) {              // Forward or Reverse motion
        motorPWM = minPWM + static_cast<int>((vel) * (maxPWM - minPWM) / maxSpeed);
        direction = "forward";
    } else if (vel < 0) {       // Reverse motion
        motorPWM = minPWM + static_cast<int>(std::abs(vel) * (maxPWM - minPWM) / maxSpeed);
        direction = "reverse";
    } else {                    // No motion
        motorPWM = minPWM;      // Motor is off
        direction = "stop";
    }      
}

void RealCarHardware::servoVelToPWM(double vel, int& servoPWM)
{
    // Define the mapping constants
    double maxSpeed = 1.0;      // Maximum speed
    int maxPWM = 2000000;       // Maximum PWM value (for max left)
    int minPWM = 1000000;       // Minimum PWM value (for max right)
    int brakePWM = 1500000;     // PWM value for straight

    // Convert speed to PWM signal
    if (vel > 0) {              // Right motion
        servoPWM = brakePWM + static_cast<int>(vel * (maxPWM - brakePWM) / maxSpeed);
    } else if (vel < 0) {       // Left motion
        servoPWM = brakePWM - static_cast<int>(std::abs(vel) * (brakePWM - minPWM) / maxSpeed);
    } else {                    // Straight
        servoPWM = brakePWM;
    }
}


// Start of Hardware Interface Stuff
hardware_interface::CallbackReturn RealCarHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  motor_subscriber_ = std::make_shared<HardwareCommandSubMotorServo>();

  motor_pub_ = std::make_shared<HardwareCommandPubMotor>();
  servo_pub_ = std::make_shared<HardwareCommandPubServo>();
  
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RealCarHardware"),
      "RealCarHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("RealCarHardware"), "Joint '%s' is a steering joint.",
        joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RealCarHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RealCarHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RealCarHardware"),
          "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RealCarHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("RealCarHardware"), "Joint '%s' is a drive joint.",
        joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RealCarHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RealCarHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RealCarHardware"),
          "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RealCarHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RealCarHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // Servo/Steering Joint is saved as this variable
  hw_interfaces_["steering"] = Joint("virtual_front_wheel_joint");

  // Motor/Traction Joint is saved as this variable
  hw_interfaces_["traction"] = Joint("virtual_rear_wheel_joint");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RealCarHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

    if (joint.first == "traction")
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY, &joint.second.state.velocity));
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RealCarHardware"), "Exported %zu state interfaces.",
    state_interfaces.size());

  for (auto s : state_interfaces)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("RealCarHardware"), "Exported state interface '%s'.",
      s.get_name().c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RealCarHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    if (joint.first == "steering")
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.second.joint_name, hardware_interface::HW_IF_POSITION,
        &joint.second.command.position));
    }
    else if (joint.first == "traction")
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
        &joint.second.command.velocity));
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RealCarHardware"), "Exported %zu command interfaces.",
    command_interfaces.size());

  for (auto i = 0u; i < command_interfaces.size(); i++)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("RealCarHardware"), "Exported command interface '%s'.",
      command_interfaces[i].get_name().c_str());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RealCarHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "Activating ...please wait...");

  // Idk what this does
  for (auto & joint : hw_interfaces_)
  {
    joint.second.state.position = 0.0;

    if (joint.first == "traction")
    {
      joint.second.state.velocity = 0.0;
      joint.second.command.velocity = 0.0;
    }

    else if (joint.first == "steering")
    {
      joint.second.command.position = 0.0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RealCarHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "Deactivating ...please wait...");
  // I don't think we ever disconnect/deactivate this hardware interface
  // so this is left blank besides the print outs
  RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RealCarHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  hw_interfaces_["steering"].state.position = feedbackAngle; // Take in rads from pico
  hw_interfaces_["traction"].state.velocity = feedbackSpeed; // Take in speed(m/s) from pico
  hw_interfaces_["traction"].state.position += feedbackSpeed * period.seconds(); // Update position based on speed in meters

 // RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "state.position: '%f'", hw_interfaces_["steering"].state.position);
 // RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "state.velocity: '%f'", hw_interfaces_["traction"].state.velocity);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type real_car ::RealCarHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::string direction;

  // RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "comVelocityBefore: '%f'", hw_interfaces_["traction"].command.velocity);
  // RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "comSteerBefore: '%f'", hw_interfaces_["steering"].command.position);
  // if (hw_interfaces_["traction"].command.velocity == 5.780000 && abs(hw_interfaces_["steering"].command.position) == 1.570796){
  //   hw_interfaces_["traction"].command.velocity = 0.0;
  //   hw_interfaces_["steering"].command.position = 0.0;
  // }
  // RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "comVelocityAfter: '%f'", hw_interfaces_["traction"].command.velocity);
  // RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "comSteerAfter: '%f'", hw_interfaces_["steering"].command.position);

  // Dividing by 20 will return the same value as the TwistMsg sent
  normalizedSpeed = hw_interfaces_["traction"].command.velocity / 20;

  // This somehow returns the same value as the TwistMsg sent
  normalizedAngle = hw_interfaces_["steering"].command.position / 3.141592 * 2;

  motorVelToPWM(normalizedSpeed, motorPWM, direction);

  if (servoPWM == 2000000 || servoPWM == 1000000){
    //motorPWM = 0;
    direction = "stop";
  }

  motor_pub_->publishSpeed(motorPWM, direction);

  servoVelToPWM(normalizedAngle, servoPWM);
  servo_pub_->publishAngle(servoPWM);

  RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "motorPWM: '%i'", motorPWM);
  RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "direction: '%s'", direction.c_str());
  RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "servoPWM: '%i'", servoPWM);
  return hardware_interface::return_type::OK;
}

}  // namespace real_car

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  real_car::RealCarHardware, hardware_interface::SystemInterface)
