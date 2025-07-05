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
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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
    nav_msgs::msg::Path path_msg_;



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

      scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(&PF::scanCallback, this, std::placeholders::_1));  // Add this

      timer_ = this->create_wall_timer(
        std::chrono::duration<double>(interval),  // 0.02 seconds = 20ms
        std::bind(&PF::timerCallback, this));

      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pf_pose", 10);
      particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pf_particles", 10);
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/pf_path", 10);
      path_msg_.header.frame_id = "map";
    
    }
  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<nav_msgs::msg::Odometry> latestOdom;
    std::unique_ptr<geometry_msgs::msg::TwistStamped> latestCmd_vel;
    std::unique_ptr<sensor_msgs::msg::Imu> latestImu;
    std::unique_ptr<nav_msgs::msg::OccupancyGrid> latestMap;
    std::unique_ptr<sensor_msgs::msg::LaserScan> latestScan;
    
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

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      latestScan = std::make_unique<sensor_msgs::msg::LaserScan>(*msg);
    }

    void timerCallback()
    {
        if (!latestCmd_vel || !latestScan)
        {
            std::string waiting_msg = "Waiting for data from:";
            if (!latestCmd_vel) waiting_msg += " cmd_vel";
            if (!latestScan) waiting_msg += " scan";
            
            RCLCPP_INFO(this->get_logger(), waiting_msg.c_str());
            return;
        }

        Eigen::Matrix<double, 2, 1> u;
        u(0) = latestCmd_vel->twist.linear.x;
        u(1) = latestCmd_vel->twist.angular.z;

        // Prediction step
        particles = move_particles(particles, u);
        
        // Update step (only if we have scan and map data)
        if (latestScan && latestMap) {
            particles = observe_particles(particles, latestScan, latestMap);
        }
        
        publishParticles();
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
  // Add noise to prevent particle degeneracy
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> pos_noise(0.0, 0.01);  // 1cm position noise
  std::normal_distribution<> angle_noise(0.0, 0.02); // ~1 degree angle noise
  
  for(auto& particle : particles){
    // Apply motion model with noise
    particle.x += u(0) * interval * cos(particle.theta) + pos_noise(gen);
    particle.y += u(0) * interval * sin(particle.theta) + pos_noise(gen);
    particle.theta += u(1) * interval + angle_noise(gen);
    
    // Normalize angle to [-pi, pi]
    while (particle.theta > M_PI) particle.theta -= 2 * M_PI;
    while (particle.theta < -M_PI) particle.theta += 2 * M_PI;
  }
  return particles;
}

double calculate_likelihood(const Particle &particle, const std::unique_ptr<sensor_msgs::msg::LaserScan>& scan, const std::unique_ptr<nav_msgs::msg::OccupancyGrid>& map){
  double x = particle.x;
  double y = particle.y;
  double theta = particle.theta;

  double log_likelihood = 0.0;
  double sigma = 0.3;  // Increase sigma to be more forgiving

  // Only use every 20th ray to avoid underflow and reduce computation
  int step = 20;
  int valid_rays = 0;
  
  for (size_t i = 0; i < scan->ranges.size(); i += step){
    // Skip invalid ranges
    if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
      continue;
    }
    
    double angle = scan->angle_min + i * scan->angle_increment;
    double global_angle = angle + theta;

    double expected_range = raycast_on_map(x, y, global_angle, map);
    double observed_range = scan->ranges[i];

    double diff = observed_range - expected_range;
    
    // Gaussian likelihood model
    log_likelihood += -(diff * diff) / (2 * sigma * sigma);
    valid_rays++;
  }
  
  // Normalize by number of valid rays
  if (valid_rays > 0) {
    log_likelihood /= valid_rays;
  }
  
  // Add offset to prevent complete underflow
  log_likelihood += 5.0;
  
  return exp(log_likelihood);
}

double raycast_on_map(double x, double y, double global_angle, const std::unique_ptr<nav_msgs::msg::OccupancyGrid>& map)
{
    double max_range = 10.0;
    double step_size = 0.02; //m
    double distance = 0.0;

    while (distance < max_range)
    {
        double ray_x = x + distance * cos(global_angle);
        double ray_y = y + distance * sin(global_angle);

        //world cord to map ind
        int map_x = static_cast<int>((ray_x - map->info.origin.position.x) / map->info.resolution);
        int map_y = static_cast<int>((ray_y - map->info.origin.position.y) / map->info.resolution);

        if (map_x < 0 || map_y < 0 || map_x >= static_cast<int>(map->info.width) || map_y >= static_cast<int>(map->info.height))
        {
            return max_range; //if pixel is outside the map
        }

        int index = map_y * map->info.width + map_x;

        if (map->data[index] > 50) //wall threshhold
        {
            return distance; //if wall is found
        }

        distance += step_size;
    }

    return max_range; //if max range is reached
}




std::vector<Particle> observe_particles(std::vector<Particle> particles, const std::unique_ptr<sensor_msgs::msg::LaserScan>& scan, const std::unique_ptr<nav_msgs::msg::OccupancyGrid>& map)
{
    if (particles.empty()) return particles;
    
    double weight_sum = 0.0;
    for(auto& particle : particles){
        particle.weight = calculate_likelihood(particle, scan, map);
        weight_sum += particle.weight;
    }
    
    if (weight_sum <= 1e-9) {
        RCLCPP_WARN(this->get_logger(), "All particle weights are zero! Skipping resampling.");
        return particles;
    }
    
    double highest_weight = 0.0;
    Particle highest_Particle;

    // Normalize weights
    for(auto& particle : particles){
        particle.weight /= weight_sum;
        if(particle.weight > highest_weight){
          highest_weight = particle.weight;
          highest_Particle = particle;  
        }
      }

    // Simple multinomial resampling with noise
    std::vector<Particle> new_particles;
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Create discrete distribution based on weights
    std::vector<double> weights;
    for (const auto& p : particles) {
        weights.push_back(p.weight);
    }
    std::discrete_distribution<> weight_dist(weights.begin(), weights.end());
    
    // Add noise distributions
    std::normal_distribution<> pos_noise(0.0, 0.05);  // 5cm position noise
    std::normal_distribution<> angle_noise(0.0, 0.1); // ~6 degree angle noise
    
    for (size_t i = 0; i < particles.size(); ++i) {
        int index = weight_dist(gen);
        Particle new_particle = particles[index];
        
        // Add small amount of noise to prevent identical particles
        new_particle.x += pos_noise(gen);
        new_particle.y += pos_noise(gen);
        new_particle.theta += angle_noise(gen);
        
        // Normalize angle
        while (new_particle.theta > M_PI) new_particle.theta -= 2 * M_PI;
        while (new_particle.theta < -M_PI) new_particle.theta += 2 * M_PI;
        
        new_particles.push_back(new_particle);
    }

    RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f", highest_Particle.x, highest_Particle.y);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";  // or "odom", depending on your TF setup

    pose_msg.pose.position.x = highest_Particle.x;
    pose_msg.pose.position.y = highest_Particle.y;
    pose_msg.pose.position.z = highest_Particle.theta;

    tf2::Quaternion q;
    q.setRPY(0, 0, highest_Particle.theta);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    path_msg_.header.stamp = this->now();
    path_msg_.poses.push_back(pose_msg);

    path_pub_->publish(path_msg_);

    pose_pub_->publish(pose_msg);
    
    return new_particles;
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

  void publishParticles()
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