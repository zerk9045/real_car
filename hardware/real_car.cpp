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

namespace real_car
{

// Create the topic that the Pi will publish to and Pico will subscribe to
HardwareCommandSubPico::HardwareCommandSubPico() : Node("pico_subscriber")
{
    pico_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "test_topic", 10, std::bind(&HardwareCommandSubPico::subscriber_callback, this, std::placeholders::_1));
}


void HardwareCommandSubPico::subscriber_callback(const std_msgs::msg::String::SharedPtr msg)
{
  latest_message_data_ = msg->data;
}

std::string HardwareCommandSubPico::getLatestMessageData()
{
  return latest_message_data_;
}

// Create the topic that the Pi will publish to and Pico will subscribe to
HardwareCommandPubMotor::HardwareCommandPubMotor() : Node("motor_publisher")
{
  motor_publisher_ = this->create_publisher<std_msgs::msg::String>("pi_motor_publishing_topic", 10);
}

// Create the topic that the Pi will publish to and Pico will subscribe to
HardwareCommandPubServo::HardwareCommandPubServo() : Node("servo_publisher")
{
  servo_publisher_ = this->create_publisher<std_msgs::msg::String>("pi_servo_publishing_topic", 10);
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

// Function for converting twist.linear.x to PWM signals
void RealCarHardware::motorVelToPWM(double vel, double normalizedAngle, int& motorPWM, std::string& direction)
{
    // Define the mapping constants
    double maxSpeed = 15.5;     // Maximum speed
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

double RealCarHardware::pwmToMotorVel(double receivedMotorPWM, std::string receivedMotorDirection)
{
    double maxSpeed = 15.5;     // Maximum speed
    int maxPWM = 2000000;       // Maximum PWM value (for max forward)
    int minPWM = 1375000;       // Minimum PWM value (for motor off)

    double newNormalizedSpeed;

    if (receivedMotorDirection == "forward") {
        newNormalizedSpeed = static_cast<double>(receivedMotorPWM - minPWM) / static_cast<double>(maxPWM - minPWM) * maxSpeed;
    } else if (receivedMotorDirection == "reverse") {
        newNormalizedSpeed = -static_cast<double>(receivedMotorPWM - minPWM) / static_cast<double>(maxPWM - minPWM) * maxSpeed;
    } else {
        newNormalizedSpeed = 0.0;
    }

    return newNormalizedSpeed;
}

double RealCarHardware::pwmToServoVel(double receivedServoPWM)
{
    double maxSpeed = 1.0;      // Maximum speed
    int maxPWM = 2000000;       // Maximum PWM value (for max left)
    int minPWM = 1000000;       // Minimum PWM value (for max right)
    int brakePWM = 1500000;     // PWM value for straight

    double newNormalizedAngle;

    if (receivedServoPWM > brakePWM) {  // Right motion
        newNormalizedAngle = static_cast<double>(receivedServoPWM - brakePWM) / static_cast<double>(maxPWM - brakePWM) * maxSpeed;
    } else if (receivedServoPWM < brakePWM) { // Left motion
        newNormalizedAngle = static_cast<double>(receivedServoPWM - brakePWM) / static_cast<double>(brakePWM - minPWM) * maxSpeed;
    } else {
        newNormalizedAngle = 0.0;
    }

    return newNormalizedAngle;
}

// Start of Hardware Interface Stuff
hardware_interface::CallbackReturn RealCarHardware::on_init(const hardware_interface::HardwareInfo & info)
{

  motor_node_ = rclcpp::Node::make_shared("motor_node");
  motor_subscriber_ = motor_node_->create_subscription<std_msgs::msg::String>(
      "pi_motor_publishing_topic", rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::String::SharedPtr motormessage) { motor_msg_ = *motormessage; });

  servo_node_ = rclcpp::Node::make_shared("servo_node");
  servo_subscriber_ = servo_node_->create_subscription<std_msgs::msg::String>(
      "pi_servo_publishing_topic", rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::String::SharedPtr servomessage) { servo_msg_ = *servomessage; });


  motor_pub_ = std::make_shared<HardwareCommandPubMotor>();
  servo_pub_ = std::make_shared<HardwareCommandPubServo>();
  pico_subscriber_ = std::make_shared<HardwareCommandSubPico>();
  
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
  if (rclcpp::ok())
  {
    rclcpp::spin_some(motor_node_);
    rclcpp::spin_some(servo_node_);
  }

  const char* motor_msg_data = motor_msg_.data.c_str();
  const char* space_pos = std::strchr(motor_msg_data, ' ');
  std::ptrdiff_t length = space_pos - motor_msg_data;
  char* endptr;
  double receivedMotorPWM = std::strtod(motor_msg_data, &endptr);
  std::string receivedMotorDirection(endptr + 1);
  //RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "Received motor message from pico: '%f'", receivedMotorPWM);
  //RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "Received motor message from pico: '%s'", receivedMotorDirection.c_str());
  
  const char* servo_msg_data = servo_msg_.data.c_str();
  const char* space_pos1 = std::strchr(servo_msg_data, ' ');
  std::ptrdiff_t length1 = space_pos1 - servo_msg_data;
  char* endptr1;
  double receivedServoPWM = std::strtod(servo_msg_data, &endptr1);
  //RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "Received servo message from pico: '%f'", receivedServoPWM);

  double newNormalizedSpeed = pwmToMotorVel(receivedMotorPWM, receivedMotorDirection);
  double newNormalizedAngle = pwmToServoVel(receivedServoPWM);

  //RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "newNormalizedSpeed: '%f'", newNormalizedSpeed);
  //RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "newNormalizedAngle: '%f'", newNormalizedAngle);

  hw_interfaces_["steering"].state.position = newNormalizedAngle * 3.141592 / 2;
  hw_interfaces_["traction"].state.velocity = (newNormalizedSpeed*20);
  hw_interfaces_["traction"].state.position += hw_interfaces_["traction"].state.velocity * period.seconds();

 // RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "state.position: '%f'", hw_interfaces_["steering"].state.position);
 // RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "state.velocity: '%f'", hw_interfaces_["traction"].state.velocity);

  // THIS WHERE WE WANT TO READ THE OUR ENCODER VALUES (IR SPEED SENSOR DATA)
  // First, read in the number of counts from Pico_Publisher Topic
  // Second, figure out the number of counts per revolution of the gearbox from Pico_Publisher Topic (this is constant, so we have calculate it by testing)
  // Third, to calculate angle in radians --> WheelAngleInRads = (number of counts)*(number of counts per revolution)
  // Basically, hw_interfaces_["traction"].state.position = WheelAngleInRads

  // The current speed of the motor needs to be fed into hw_interfaces_["traction"].state.velocity
  // We need to figure out the corresponding counts per second to the max, neutral, and min speed of the motor.

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type real_car ::RealCarHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::string direction;

  RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "comVelocityBefore: '%f'", hw_interfaces_["traction"].command.velocity);
  RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "comSteerBefore: '%f'", hw_interfaces_["steering"].command.position);
  if (hw_interfaces_["traction"].command.velocity == 5.780000 && abs(hw_interfaces_["steering"].command.position) == 1.570796){
    hw_interfaces_["traction"].command.velocity = 0.0;
    hw_interfaces_["steering"].command.position = 0.0;
  }
  RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "comVelocityAfter: '%f'", hw_interfaces_["traction"].command.velocity);
  RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "comSteerAfter: '%f'", hw_interfaces_["steering"].command.position);

  // Dividing by 20 will return the same value as the TwistMsg sent
  normalizedSpeed = hw_interfaces_["traction"].command.velocity / 20;

  // This somehow returns the same value as the TwistMsg sent
  normalizedAngle = hw_interfaces_["steering"].command.position / 3.141592 * 2;

  motorVelToPWM(normalizedSpeed, normalizedAngle, motorPWM, direction);

  if (motorPWM == 1386653 && direction == "forward"){
    motorPWM = 0;
    direction = "stop";
  }

  motor_pub_->publishSpeed(motorPWM, direction);

  servoVelToPWM(normalizedAngle, servoPWM);
  servo_pub_->publishAngle(servoPWM);
  // RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "normalizedAngle: '%f'", normalizedAngle);
  // RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "motorPWM: '%i'", motorPWM);
  // RCLCPP_INFO(rclcpp::get_logger("RealCarHardware"), "direction: '%s'", direction.c_str());

  return hardware_interface::return_type::OK;
}

}  // namespace real_car

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  real_car::RealCarHardware, hardware_interface::SystemInterface)
