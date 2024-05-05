// Created by Team 6 for EEC 195AB, Spring 2024
// Adrien Salvador, Raphael Bret-Mounet, Gabriel Castellanos

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
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "real_car/visibility_control.h"

namespace real_car
{
class HardwareCommandSubPico : public rclcpp::Node
{
  public:
    HardwareCommandSubPico();
    void subscriber_callback(const std_msgs::msg::String::SharedPtr msg);
    std::string getLatestMessageData();
  private:
    std::string latest_message_data_; 
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pico_subscriber_;

};

// This is the node definition for the publisher that the Pi publishes to and Pico subscribes to for motor speeds
class HardwareCommandPubMotor : public rclcpp::Node
{
  public:
    HardwareCommandPubMotor();
    void publishSpeed(int speed, std::string direction);
  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_publisher_;

};

// This is the node definition for the publisher that the Pi publishes to and Pico subscribes to servo speeds
class HardwareCommandPubServo : public rclcpp::Node
{
  public:
    HardwareCommandPubServo();
    void publishAngle(int angle);
  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr servo_publisher_;

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

  std::shared_ptr<HardwareCommandPubMotor> motor_pub_;    //make the publisher node a member
  std::shared_ptr<HardwareCommandPubServo> servo_pub_;    //make the publisher node a member  
  std::shared_ptr<HardwareCommandSubPico> pico_subscriber_;
  // function defintion to convert normalized twist.linear.x to pwm
  void motorVelToPWM(double vel, double normalizedAngle, int& motorPWM, std::string& direction);
  void servoVelToPWM(double vel, int& servoPWM);
  double pwmToMotorVel(double receivedMotorPWM, std::string receivedMotorDirection);
  double pwmToServoVel(double receivedServoPWM);
private:

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motor_subscriber_;
  rclcpp::Node::SharedPtr motor_node_;
  std_msgs::msg::String motor_msg_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr servo_subscriber_;
  rclcpp::Node::SharedPtr servo_node_;
  std_msgs::msg::String servo_msg_;

  // std::vector<std::tuple<std::string, double, double>>
  // hw_interfaces_;  // name of joint, state, command
  std::map<std::string, Joint> hw_interfaces_;
  int motorPWM;
  int servoPWM;  
  double normalizedSpeed;
  double normalizedAngle;
  double newNormalizedSpeed;
  double newNormalizedAngle;  
  double speed2publish;
  double angle2publish;
};

}  // namespace real_car

#endif  // REAL_CAR__CARLIKEBOT_SYSTEM_HPP_
