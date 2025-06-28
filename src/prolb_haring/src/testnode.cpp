#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

class KF:public rclcpp::Node{
  public:
    KF():Node("kalman_node"){
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 
        10,
        std::bind(&KF::odomCallback, this, std::placeholders::_1));
    
      cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&KF::cmd_velCallback, this, std::placeholders::_1));
      
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu",
        10,
        std::bind(&KF::imuCallback, this, std::placeholders::_1));

      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),  //refresh rate
        std::bind(&KF::timerCallback, this)
      );
    }
  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<nav_msgs::msg::Odometry> latestOdom;
    std::unique_ptr<geometry_msgs::msg::Twist> latestCmd_vel;
    std::unique_ptr<sensor_msgs::msg::Imu> latestImu;
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      latestOdom = std::make_unique<nav_msgs::msg::Odometry>(*msg);
    }

    void cmd_velCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      latestCmd_vel = std::make_unique<geometry_msgs::msg::Twist>(*msg);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
      latestImu = std::make_unique<sensor_msgs::msg::Imu>(*msg);
    }

    void timerCallback()
    {
      if (!latestOdom || !latestCmd_vel || !latestImu)
      {
        RCLCPP_INFO(this->get_logger(), "Waiting for all data...");
        return;
      }

      RCLCPP_INFO(this->get_logger(), 
        "Odom x: %.2f, Cmd_vel x: %.2f, IMU acc x: %.2f",
        latestOdom->pose.pose.position.x,
        latestCmd_vel->linear.x,
        latestImu->linear_acceleration.x);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KF>());
  rclcpp::shutdown();
  return 0;
}