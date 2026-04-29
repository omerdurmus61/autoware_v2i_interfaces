#include "driver_approved_virtual_traffic_light/driver_approved_virtual_traffic_light_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      driver_approved_virtual_traffic_light::DriverApprovedVirtualTrafficLightNode>());
  rclcpp::shutdown();
  return 0;
}
