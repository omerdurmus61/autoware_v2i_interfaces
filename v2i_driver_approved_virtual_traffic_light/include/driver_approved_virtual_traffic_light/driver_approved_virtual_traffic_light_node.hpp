#ifndef DRIVER_APPROVED_VIRTUAL_TRAFFIC_LIGHT__DRIVER_APPROVED_VIRTUAL_TRAFFIC_LIGHT_NODE_HPP_
#define DRIVER_APPROVED_VIRTUAL_TRAFFIC_LIGHT__DRIVER_APPROVED_VIRTUAL_TRAFFIC_LIGHT_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "raptor_dbw_msgs/msg/driver_input_report.hpp"
#include "tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp"

namespace driver_approved_virtual_traffic_light
{

class DriverApprovedVirtualTrafficLightNode : public rclcpp::Node
{
public:
  explicit DriverApprovedVirtualTrafficLightNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void driverInputCallback(const raptor_dbw_msgs::msg::DriverInputReport::SharedPtr msg);
  void publishTimerCallback();
  void logParameters() const;

  std::string driver_input_topic_;
  std::string virtual_traffic_light_state_topic_;
  std::string virtual_traffic_light_type_;
  std::string virtual_traffic_light_id_;
  double approval_duration_sec_;
  double publish_rate_hz_;
  bool is_finalized_;

  bool previous_a_;
  bool approval_active_;
  rclcpp::Time approval_until_time_;

  rclcpp::Subscription<raptor_dbw_msgs::msg::DriverInputReport>::SharedPtr driver_input_sub_;
  rclcpp::Publisher<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace driver_approved_virtual_traffic_light

#endif  // DRIVER_APPROVED_VIRTUAL_TRAFFIC_LIGHT__DRIVER_APPROVED_VIRTUAL_TRAFFIC_LIGHT_NODE_HPP_
