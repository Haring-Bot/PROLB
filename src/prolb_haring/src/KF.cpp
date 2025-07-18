#include <memory>
#include <chrono>
#include <algorithm>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class KF:public rclcpp::Node{
  public:
    double interval = 0.2; //filter update rate

    KF():Node("kalman_node"){
      mu << -2.0, -0.5, 0.0, 0.0, 0.0, 0.0; //initial state
      sigma = Eigen::Matrix<double, 6, 6>::Identity(); //initial covariance

      //subscribe to sensor data
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

      //initialize TF2 buffer and listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      timer_ = this->create_wall_timer(
        std::chrono::duration<double>(interval),  // 0.02 seconds = 20ms
        std::bind(&KF::timerCallback, this));

      //publishers
      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/kf_pose", 10);
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/kf_path", 10);
      path_msg_.header.frame_id = "map";
      true_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/ground_truth_path", 10);
      true_path_msg_.header.frame_id = "map";
    }
  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    //TF2 for getting true pose
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr true_path_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<nav_msgs::msg::Odometry> latestOdom;
    std::unique_ptr<geometry_msgs::msg::TwistStamped> latestCmd_vel;
    std::unique_ptr<sensor_msgs::msg::Imu> latestImu;
    nav_msgs::msg::Path path_msg_;
    nav_msgs::msg::Path true_path_msg_;
    std::string target_frame_ = "base_link";
    std::string source_frame_ = "map";
    
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
      //check if all sensor data is available
      if (!latestOdom || !latestCmd_vel || !latestImu)
      {
        std::string waiting_msg = "Waiting for data from:";
        if (!latestOdom) waiting_msg += " odometry";
        if (!latestCmd_vel) waiting_msg += " cmd_vel";
        if (!latestImu) waiting_msg += " imu";
        
        RCLCPP_INFO(this->get_logger(), waiting_msg.c_str());
        return;
      }

      //run kalman filter
      calculateKalman();

      //prepare pose message
      geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
      pose_msg.header.stamp = this->now();
      pose_msg.header.frame_id = "map";

      pose_msg.pose.pose.position.x = mu(0);
      pose_msg.pose.pose.position.y = mu(1);
      pose_msg.pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, mu(2));
      pose_msg.pose.pose.orientation.x = q.x();
      pose_msg.pose.pose.orientation.y = q.y();
      pose_msg.pose.pose.orientation.z = q.z();
      pose_msg.pose.pose.orientation.w = q.w();

      //set covariance matrix
      for (int i = 0; i < 6; ++i)
      {
        for (int j = 0; j < 6; ++j)
        {
          pose_msg.pose.covariance[i * 6 + j] = sigma(i, j);
        }
      }

      //create PoseStamped from PoseWithCovarianceStamped
      geometry_msgs::msg::PoseStamped path_pose;
      path_pose.header = pose_msg.header;
      path_pose.pose = pose_msg.pose.pose;  // Extract the pose from pose.pose

      path_msg_.header.stamp = this->now();
      path_msg_.poses.push_back(path_pose);

      path_pub_->publish(path_msg_);

      //get robot true pose from TF2
      geometry_msgs::msg::PoseStamped true_path_pose;
      true_path_pose.header = pose_msg.header;
      
      if (getRobotTruePoseFromTF(true_path_pose.pose))
      {
        true_path_msg_.header.stamp = this->now();
        true_path_msg_.poses.push_back(true_path_pose);
        true_path_pub_->publish(true_path_msg_);
      }
      else
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Could not get transform from '%s' to '%s'", 
                            source_frame_.c_str(), target_frame_.c_str());
      }

      pose_pub_->publish(pose_msg);
      
      //RCLCPP_INFO(this->get_logger(), "Publishing pose at time: %u.%u",
      //pose_msg.header.stamp.sec,
      //pose_msg.header.stamp.nanosec);

      //RCLCPP_INFO(this->get_logger(), "Estimated Pose -> x: %.3f, y: %.3f, theta: %.3f", mu(0), mu(1), mu(2));
      //RCLCPP_INFO(this->get_logger(), "Simplified uncertainty (trace of covariance): %.6f", sigma.trace());

    }

    // struct kalmanReturn{
    //   Eigen::Matrix<double, 6, 1> mu;
    //   Eigen::Matrix<double, 6, 6> sigma;
    // };
    Eigen::Matrix<double, 6, 1> mu; //state vector
    Eigen::Matrix<double, 6, 6> sigma; //covariance matrix

    void calculateKalman()
    {
      Eigen::Matrix<double, 2, 1> u; //control input
      Eigen::Matrix<double, 6, 1> z; //measurement vector

      //current state
      double x = this->mu(0);
      double y = this->mu(1);
      double theta = this->mu(2);
      double x_vel = this->mu(3);
      double y_vel = this->mu(4);
      double omega = this->mu(5);

      //control input from cmd_vel
      u(0) = latestCmd_vel->twist.linear.x;
      u(1) = latestCmd_vel->twist.angular.z;

      //extract yaw from IMU quaternion
      tf2::Quaternion qImu(
        latestImu->orientation.x,
        latestImu->orientation.y,
        latestImu->orientation.z,
        latestImu->orientation.w
      );
      double roll, pitch, thetaImu;
      tf2::Matrix3x3(qImu).getRPY(roll, pitch, thetaImu);

      //construct measurement vector
      z(0) = x + (latestOdom->twist.twist.linear.x * cos(thetaImu) * interval);
      z(1) = y + (latestOdom->twist.twist.linear.x * sin(thetaImu) * interval);
      z(2) = thetaImu;
      z(3) = latestImu->linear_acceleration.x * cos(thetaImu) * interval;
      z(4) = latestImu->linear_acceleration.x * sin(thetaImu) * interval;
      z(5) = latestImu->angular_velocity.z;

      //RCLCPP_INFO(this->get_logger(), "Estimated Pose according to cmd_vel pred -> x: %.3f, y: %.3f, theta: %.3f", z(0), z(1), z(2));

      //state transition matrix
      Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
      A(0,3) = interval;
      A(1,4) = interval;
      A(2,5) = interval;

      //control input matrix
      Eigen::Matrix<double, 6, 2> B;
      B.setZero();
      B(3, 0) = cos(theta);
      B(4, 0) = sin(theta);
      B(5, 1) = 1;

      //predict state
      Eigen::Matrix<double, 6, 1> muPred = A * mu + B * u;    //apply motion model

      //load process noise covariance from yaml
      Eigen::Matrix<double, 6, 6> R = loadMatrixFromYaml("settings.yaml", "R"); // Cov linear model

      //predict covariance
      Eigen::Matrix<double, 6, 6> sigmaPred = A * this->sigma * A.transpose() + R;    //predicted covariance

      //observation matrix and measurement noise
      Eigen::Matrix<double, 6, 6> C = loadMatrixFromYaml("settings.yaml", "C");
      //printMatrix(C, "C");
      Eigen::Matrix<double, 6, 6> Q = loadMatrixFromYaml("settings.yaml", "Q");
      
      //kalman gain
      Eigen::Matrix<double, 6, 6> kalmanGain = sigmaPred * C.transpose() * (C * sigmaPred * C.transpose() + Q).inverse();     //calculate Kalman Gain

      //update state and covariance
      Eigen::Matrix<double, 6, 1> muCor = muPred + kalmanGain * (z - C * muPred);       //correct prediction based on Kalman Gain
      Eigen::Matrix<double, 6, 6> sigmaCor = (Eigen::MatrixXd::Identity(6,6) - kalmanGain * C) * sigmaPred;    //correct covariance based on Kalman Gain
    
      //update class variables
      this->mu = muCor;
      this->sigma = sigmaCor;

      //printMatrix(kalmanGain, "kalmanGain");

      return;
    }

    //convert quaternion to yaw angle
    double quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
    {
      tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
      return yaw;
    }

    //get robot true pose from TF
    bool getRobotTruePoseFromTF(geometry_msgs::msg::Pose& robot_pose)
    {
      try
      {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_->lookupTransform(
          source_frame_, target_frame_, tf2::TimePointZero);
        
        //convert transform to pose
        robot_pose.position.x = transform_stamped.transform.translation.x;
        robot_pose.position.y = transform_stamped.transform.translation.y;
        robot_pose.position.z = transform_stamped.transform.translation.z;
        robot_pose.orientation = transform_stamped.transform.rotation;
        
        return true;
      }
      catch (tf2::TransformException &ex)
      {
        return false;
      }
    }

    //debug function to print matrices
    void printMatrix(const Eigen::MatrixXd& matrix, const std::string& name = "Matrix")
    {
      RCLCPP_INFO(this->get_logger(), "%s (%dx%d):", name.c_str(), 
                  static_cast<int>(matrix.rows()), static_cast<int>(matrix.cols()));
      
      for (int i = 0; i < matrix.rows(); ++i)
      {
        std::string row_str = "";
        for (int j = 0; j < matrix.cols(); ++j)
        {
          row_str += std::to_string(matrix(i, j));
          if (j < matrix.cols() - 1) row_str += ", ";
        }
        RCLCPP_INFO(this->get_logger(), "  [%s]", row_str.c_str());    }
  }

  //load matrix from yaml configuration file
  Eigen::MatrixXd loadMatrixFromYaml(const std::string & yaml_file, const std::string & matrix_name)
  {
    try
    {
      YAML::Node config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("prolb_haring") + "/config/" + yaml_file);
      if (!config[matrix_name])
      {
        throw std::runtime_error("Matrix '" + matrix_name + "' not found in YAML file.");
      }

      auto matrix_node = config[matrix_name];
      int rows = matrix_node["rows"].as<int>();
      int cols = matrix_node["cols"].as<int>();

      Eigen::MatrixXd matrix(rows, cols);
      auto data_node = matrix_node["data"];

      for (int i = 0; i < rows; ++i)
      {
        for (int j = 0; j < cols; ++j)
        {
          matrix(i, j) = data_node[i][j].as<double>();
        }
      }

      return matrix;
    }
    catch (const YAML::Exception & e)
    {
      throw std::runtime_error("Failed to load YAML file: " + std::string(e.what()));
    }
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KF>());
  rclcpp::shutdown();
  return 0;
}