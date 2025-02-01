#include "pf_localization/pf_localization.hpp"

namespace pf_localization
{
  ParticleFilterLocalization::ParticleFilterLocalization()
  : Node("pf_localization_node"), rate_hz_(30)
  {
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "diff_drive/scan", 10, std::bind(&ParticleFilterLocalization::laser_callback, this, _1));
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "diff_drive/odometry", 10, std::bind(&ParticleFilterLocalization::odom_callback, this, _1));
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", 10, std::bind(&ParticleFilterLocalization::map_callback, this, _1));
    timer_ = this->create_wall_timer(
    std::chrono::microseconds((int) 1.0e6 / rate_hz_), std::bind(&ParticleFilterLocalization::loop, this));
  }

  ParticleFilterLocalization::~ParticleFilterLocalization()
  {
    
  }

  void ParticleFilterLocalization::loop()
  {
    this->publish_pose();
  }
  void ParticleFilterLocalization::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_ = msg;
  }
  void ParticleFilterLocalization::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = msg;
  }
  void ParticleFilterLocalization::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
  }
  void ParticleFilterLocalization::publish_pose()
  {
    auto message = geometry_msgs::msg::PoseStamped();
    pose_publisher_->publish(message);
  }
}