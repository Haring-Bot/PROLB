#include <memory>
#include <chrono>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class KF:public rclcpp::Node{
  public:
    double interval = 0.02;


    KF():Node("kalman_node"){
      mu << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      sigma = Eigen::Matrix<double, 6, 6>::Identity();

      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 
        10,
        std::bind(&KF::odomCallback, this, std::placeholders::_1));
    
      cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel",
        10,
        std::bind(&KF::cmd_velCallback, this, std::placeholders::_1));
      
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu",
        10,
        std::bind(&KF::imuCallback, this, std::placeholders::_1));

      timer_ = this->create_wall_timer(
        std::chrono::duration<double>(interval),  // 0.02 seconds = 20ms
        std::bind(&KF::timerCallback, this)
      );
    }
  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<nav_msgs::msg::Odometry> latestOdom;
    std::unique_ptr<geometry_msgs::msg::TwistStamped> latestCmd_vel;
    std::unique_ptr<sensor_msgs::msg::Imu> latestImu;
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      latestOdom = std::make_unique<nav_msgs::msg::Odometry>(*msg);
    }

    void cmd_velCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
      latestCmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>(*msg);
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
      Eigen::Matrix<double, 2, 1> u;
      u(0) = latestCmd_vel->twist.linear.x;
      u(1) = latestCmd_vel->twist.angular.z;

      Eigen::Matrix<double, 6, 1> z;
      z(0) = latestOdom->pose.pose.position.x;
      z(1) = latestOdom->pose.pose.position.y;
      z(2) = quaternionToYaw(latestOdom->pose.pose.orientation);
      z(3) = latestOdom->twist.twist.linear.x;
      z(4) = latestOdom->twist.twist.linear.y;
      z(5) = latestOdom->twist.twist.angular.z;

      calculateKalman(mu, sigma, u, z);

      RCLCPP_INFO(this->get_logger(), "Estimated Pose -> x: %.3f, y: %.3f, theta: %.3f", mu(0), mu(1), mu(2));
      RCLCPP_INFO(this->get_logger(), "Simplified uncertainty (trace of covariance): %.6f", sigma.trace());

    }

    // struct kalmanReturn{
    //   Eigen::Matrix<double, 6, 1> mu;
    //   Eigen::Matrix<double, 6, 6> sigma;
    // };
    Eigen::Matrix<double, 6, 1> mu;
    Eigen::Matrix<double, 6, 6> sigma;

    void calculateKalman(const Eigen::Matrix<double, 2, 1> u, const Eigen::Matrix<double, 6, 1> z)
    {
      double x = this->mu(0);
      double y = this->mu(1);
      double theta = this->mu(2);
      double x_vel = this->mu(3);
      double y_vel = this->mu(4);
      double theta_vel = this->mu(5);

      Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
      A(0,3) = interval;
      A(1,4) = interval;
      A(2,5) = interval;

      Eigen::Matrix<double, 6, 2> B;
      B.setZero();

      B(0, 0) = interval * cos(theta);
      B(1, 0) = interval * sin(theta);
      B(2, 1) = interval;

      B(3, 0) = cos(theta);
      B(4, 0) = sin(theta);
      B(5, 1) = 1;

      Eigen::Matrix<double, 6, 1> muPred = A * mu + B * u;    //apply motion model

      Eigen::Matrix<double, 6, 6> R = 0.1 * Eigen::MatrixXd::Identity(6,6); // Cov linear model

      Eigen::Matrix<double, 6, 6> sigmaPred = A * this->sigma * A.transpose() + R;    //predicted covariance

      Eigen::Matrix<double, 6, 6> C = Eigen::MatrixXd::Identity(6,6);
      Eigen::Matrix<double, 6, 6> Q = 0.1 * Eigen::MatrixXd::Identity(6,6);
      Eigen::Matrix<double, 6, 6> kalmanGain = sigmaPred * C.transpose() * (C * sigmaPred * C.transpose() + Q).inverse();     //calculate Kalman Gain

      Eigen::Matrix<double, 6, 1> muCor = muPred + kalmanGain * (z - C * muPred);       //correct prediction based on Kalman Gain
      Eigen::Matrix<double, 6, 6> sigmaCor = (Eigen::MatrixXd::Identity(6,6) - kalmanGain * C) * sigmaPred;    //correct covariance based on Kalman Gain
    
      this->mu = muCor;
      this->sigma = sigmaCor;

      return;
    }

    double quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
    {
      tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
      return yaw;
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KF>());
  rclcpp::shutdown();
  return 0;
}