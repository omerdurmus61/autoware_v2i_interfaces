#include "driver_approved_virtual_traffic_light/driver_approved_virtual_traffic_light_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "tier4_v2x_msgs/msg/virtual_traffic_light_state.hpp"

namespace driver_approved_virtual_traffic_light
{

using namespace std::chrono_literals;

DriverApprovedVirtualTrafficLightNode::DriverApprovedVirtualTrafficLightNode(
  const rclcpp::NodeOptions & options)
: Node("driver_approved_virtual_traffic_light_node", options),
  previous_a_(false),
  approval_active_(false),
  approval_until_time_(0, 0, this->get_clock()->get_clock_type())
{
  driver_input_topic_ = this->declare_parameter<std::string>(
    "driver_input_topic", "/raptor_dbw_interface/driver_input_report");
  virtual_traffic_light_state_topic_ = this->declare_parameter<std::string>(
    "virtual_traffic_light_state_topic", "/awapi/tmp/virtual_traffic_light_states");
  virtual_traffic_light_type_ = this->declare_parameter<std::string>(
    "virtual_traffic_light_type", "virtual");
  virtual_traffic_light_id_ = this->declare_parameter<std::string>(
    "virtual_traffic_light_id", "99");
  approval_duration_sec_ = this->declare_parameter<double>("approval_duration_sec", 5.0);
  publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);
  is_finalized_ = this->declare_parameter<bool>("is_finalized", false);

  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "publish_rate_hz must be > 0.0. Falling back to 10.0 Hz.");
    publish_rate_hz_ = 10.0;
  }

  if (approval_duration_sec_ < 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "approval_duration_sec must be >= 0.0. Falling back to 5.0 sec.");
    approval_duration_sec_ = 5.0;
  }

  logParameters();

  driver_input_sub_ = this->create_subscription<raptor_dbw_msgs::msg::DriverInputReport>(
    driver_input_topic_,
    rclcpp::QoS{10},
    std::bind(
      &DriverApprovedVirtualTrafficLightNode::driverInputCallback,
      this,
      std::placeholders::_1));

  state_pub_ = this->create_publisher<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>(
    virtual_traffic_light_state_topic_,
    rclcpp::QoS{10});

  const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
    std::bind(&DriverApprovedVirtualTrafficLightNode::publishTimerCallback, this));
}

void DriverApprovedVirtualTrafficLightNode::driverInputCallback(
  const raptor_dbw_msgs::msg::DriverInputReport::SharedPtr msg)
{
  const bool current_a = msg->steer_wheel_button_a;
  const auto now = this->now();

  if (current_a && !previous_a_) {
    approval_active_ = true;
    approval_until_time_ = now + rclcpp::Duration::from_seconds(approval_duration_sec_);

    RCLCPP_INFO(
      this->get_logger(),
      "Driver approval received. Virtual traffic light approval active for %.2f seconds. "
      "type=%s id=%s",
      approval_duration_sec_,
      virtual_traffic_light_type_.c_str(),
      virtual_traffic_light_id_.c_str());
  }

  previous_a_ = current_a;
}

void DriverApprovedVirtualTrafficLightNode::publishTimerCallback()
{
  const auto now = this->now();

  if (approval_active_ && now > approval_until_time_) {
    approval_active_ = false;
    RCLCPP_INFO(this->get_logger(), "Virtual traffic light approval expired.");
  }

  tier4_v2x_msgs::msg::VirtualTrafficLightState state_msg;
  state_msg.stamp = now;
  state_msg.type = virtual_traffic_light_type_;
  state_msg.id = virtual_traffic_light_id_;
  state_msg.approval = approval_active_;
  state_msg.is_finalized = is_finalized_;

  tier4_v2x_msgs::msg::VirtualTrafficLightStateArray array_msg;
  array_msg.stamp = now;
  array_msg.states.push_back(state_msg);

  state_pub_->publish(array_msg);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    1000,
    "Publishing virtual traffic light approval: approval=%s type=%s id=%s finalized=%s",
    approval_active_ ? "true" : "false",
    virtual_traffic_light_type_.c_str(),
    virtual_traffic_light_id_.c_str(),
    is_finalized_ ? "true" : "false");
}

void DriverApprovedVirtualTrafficLightNode::logParameters() const
{
  RCLCPP_INFO(
    this->get_logger(),
    "Starting driver approved virtual traffic light node with parameters: "
    "driver_input_topic=%s, virtual_traffic_light_state_topic=%s, "
    "virtual_traffic_light_type=%s, virtual_traffic_light_id=%s, "
    "approval_duration_sec=%.2f, publish_rate_hz=%.2f, is_finalized=%s",
    driver_input_topic_.c_str(),
    virtual_traffic_light_state_topic_.c_str(),
    virtual_traffic_light_type_.c_str(),
    virtual_traffic_light_id_.c_str(),
    approval_duration_sec_,
    publish_rate_hz_,
    is_finalized_ ? "true" : "false");
}

}  // namespace driver_approved_virtual_traffic_light
