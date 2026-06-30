#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class CameraInfoFrameRepublisher : public rclcpp::Node
{
public:
  CameraInfoFrameRepublisher()
  : Node("camera_info_frame_republisher")
  {
    input_camera_info_topic_ = declare_parameter<std::string>(
      "input_camera_info_topic", "/sensing/cam2/camera_info");
    output_camera_info_topic_ = declare_parameter<std::string>(
      "output_camera_info_topic", "/sensing/cam2/camera_info_optical");
    target_frame_id_ = declare_parameter<std::string>(
      "target_frame_id", "vimbax_camera_DEV_000F315E05DE_optical_link");

    publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(output_camera_info_topic_, 10);
    subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      input_camera_info_topic_,
      10,
      std::bind(&CameraInfoFrameRepublisher::cameraInfoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Republishing CameraInfo from '%s' to '%s' with frame_id '%s'",
      input_camera_info_topic_.c_str(),
      output_camera_info_topic_.c_str(),
      target_frame_id_.c_str());
  }

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) const
  {
    auto republished_msg = *msg;
    republished_msg.header.frame_id = target_frame_id_;
    publisher_->publish(republished_msg);
  }

  std::string input_camera_info_topic_;
  std::string output_camera_info_topic_;
  std::string target_frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoFrameRepublisher>());
  rclcpp::shutdown();
  return 0;
}
