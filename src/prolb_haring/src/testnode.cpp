#include <memory>
#include <chrono>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

class KF:public rclcpp::Node{
  public:
    double interval = 0.02;

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
        std::chrono::duration<double>(interval),  // 0.02 seconds = 20ms
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
    
    Eigen::Matrix<double, 6, 1> muPrev;
    
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
    }

    void calculateKalman(Eigen::Matrix<double, 6, 1> mu, Eigen::Matrix<double, 6, 6> sigma, Eigen::Matrix<double, 2, 1> u, Eigen::Matrix<double, 2, 1> z)
    {
      double x = mu(0);
      double y = mu(1);
      double theta = mu(2);
      double x_vel = mu(3);
      double y_vel = mu(4);
      double theta_vel = mu(5);

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

      Eigen::Matrix2d R;  //Cov linear model
      R.setZero();
      R(0,0) = 0.01;
      R(1,1) = 0.01;

      Eigen::Matrix<double, 6, 6> sigmaPred = A * sigma * A.transpose() + R;    //predicted covariance

      Eigen::Matrix<double, 6, 6> C = (1.0/interval) * Eigen::MatrixXd::Identity(6,6);
      Eigen::Matrix<double, 6, 6> Q = 0.1 * Eigen::MatrixXd::Identity(6,6);
      Eigen::Matrix<double, 6, 6> kalmanGain = sigmaPred * C.transpose() * (C * sigmaPred * C.transpose() + Q).inverse();     //calculate Kalman Gain

      Eigen::Matrix<double, 6, 1> muCor = muPred + kalmanGain * (z - C * muPred);       //correct prediction based on Kalman Gain
      Eigen::Matrix<double, 6, 6> sigmaCor = (Eigen::MatrixXd::Identity(6,6) - kalmanGain * C) * sigmaPred;    //correct covariance based on Kalman Gain
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KF>());
  rclcpp::shutdown();
  return 0;
}