#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomSubscriber : public rclcpp::Node
{
public:
  OdomSubscriber()
  : Node("odom_subscriber")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, 
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "Received odom: Position: [%f, %f, %f]", 
                   msg->pose.pose.position.x, 
                   msg->pose.pose.position.y,
                   msg->pose.pose.position.z);
      });
    RCLCPP_INFO(this->get_logger(), "Odom subscriber node initialized");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomSubscriber>());
  rclcpp::shutdown();
  return 0;
}