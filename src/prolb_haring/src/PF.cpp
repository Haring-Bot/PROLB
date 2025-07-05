#include <memory>
#include <chrono>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <random>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class PF:public rclcpp::Node{
  public:
    struct Particle{
        double x;
        double y;
        double theta;
        double weight = 1.0;  // Initialize with equal weights
    };

    Eigen::Matrix<double, 6, 1> mu;
    Eigen::Matrix<double, 6, 6> sigma;
    double interval = 0.2;
    std::vector<Particle> particles;
    const int amount_particles = 1000;



    PF():Node("particle_filter_node"){

      mu << -2.0, -0.5, 0.0, 0.0, 0.0, 0.0;
      sigma = Eigen::Matrix<double, 6, 6>::Identity();

      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 
        10,
        std::bind(&PF::odomCallback, this, std::placeholders::_1));
    
      cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel",
        10,
        std::bind(&PF::cmd_velCallback, this, std::placeholders::_1));
      
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu",
        10,
        std::bind(&PF::imuCallback, this, std::placeholders::_1));

      map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        10,
        std::bind(&PF::mapCallback, this, std::placeholders::_1));

      timer_ = this->create_wall_timer(
        std::chrono::duration<double>(interval),  // 0.02 seconds = 20ms
        std::bind(&PF::timerCallback, this));

      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pf_pose", 10);
      particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pf_particles", 10);  // Add this
    
    }
  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<nav_msgs::msg::Odometry> latestOdom;
    std::unique_ptr<geometry_msgs::msg::TwistStamped> latestCmd_vel;
    std::unique_ptr<sensor_msgs::msg::Imu> latestImu;
    std::unique_ptr<nav_msgs::msg::OccupancyGrid> latestMap; 
    
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

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
      latestMap = std::make_unique<nav_msgs::msg::OccupancyGrid>(*msg);
        
      // Initialize particles when we first receive the map
      if (particles.empty()) {
          particles = initialize_particles(amount_particles, *latestMap);
          RCLCPP_INFO(this->get_logger(), "Initialized %zu particles", particles.size());
      }
    }

    void timerCallback()
    {
      if (!latestOdom || !latestCmd_vel || !latestImu)
      {
        std::string waiting_msg = "Waiting for data from:";
        if (!latestOdom) waiting_msg += " odometry";
        if (!latestCmd_vel) waiting_msg += " cmd_vel";
        if (!latestImu) waiting_msg += " imu";
        
        RCLCPP_INFO(this->get_logger(), waiting_msg.c_str());
        return;
      }

      Eigen::Matrix<double, 2, 1> u;
      u(0) = latestCmd_vel->twist.linear.x;
      u(1) = latestCmd_vel->twist.angular.z;

      particles = move_particles(particles, u);
      publishParticles(particles);



    }

std::vector<Particle> initialize_particles(int n_particles, const nav_msgs::msg::OccupancyGrid& map){
    std::vector<Particle> particles;

    // Precompute free cells
    std::vector<std::pair<int, int>> free_cells;
    int width = map.info.width;
    int height = map.info.height;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = x + y * width;
            if (map.data[index] == 0) { // Free cell
                free_cells.emplace_back(x, y);
            }
        }
    }

    if (free_cells.empty()) {
        throw std::runtime_error("Map has no free cells!");
    }

    // Random generators
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> cell_dist(0, free_cells.size() - 1);
    std::uniform_real_distribution<> angle_dist(0.0, 2 * M_PI);

    for (int i = 0; i < n_particles; ++i) {
        auto [cell_x, cell_y] = free_cells[cell_dist(gen)];

        // Convert cell to world coordinates
        double world_x = map.info.origin.position.x + (cell_x + 0.5) * map.info.resolution;
        double world_y = map.info.origin.position.y + (cell_y + 0.5) * map.info.resolution;

        double theta = angle_dist(gen);

        particles.push_back(Particle{world_x, world_y, theta});
    }

    return particles;
}

std::vector<Particle> move_particles(std::vector<Particle> particles, Eigen::Matrix<double, 2, 1> u){
  for(auto& particle : particles){
    particle.x += u(0) * interval * cos(particle.theta);
    particle.y += u(0) * interval * sin(particle.theta);
    particle.theta += u(1) * interval;
    
    // Normalize angle to [-pi, pi]
    while (particle.theta > M_PI) particle.theta -= 2 * M_PI;
    while (particle.theta < -M_PI) particle.theta += 2 * M_PI;
    }
  return particles;
}


  double quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
  {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
  }

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
      RCLCPP_INFO(this->get_logger(), "  [%s]", row_str.c_str());
    }
  }

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

  void publishParticles(std::vector<Particle> particles)
  {
    if (particles.empty()) return;
      
      geometry_msgs::msg::PoseArray particles_msg;
      particles_msg.header.stamp = this->get_clock()->now();
      particles_msg.header.frame_id = "map";
      
      for (const auto& particle : particles){
          geometry_msgs::msg::Pose pose;
          pose.position.x = particle.x;
          pose.position.y = particle.y;
          pose.position.z = 0.0;
          
          // Convert theta to quaternion
          tf2::Quaternion q;
          q.setRPY(0, 0, particle.theta);
          pose.orientation.x = q.x();
          pose.orientation.y = q.y();
          pose.orientation.z = q.z();
          pose.orientation.w = q.w();
          
          particles_msg.poses.push_back(pose);
      }
      
      particles_pub_->publish(particles_msg);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PF>());
  rclcpp::shutdown();
  return 0;
}