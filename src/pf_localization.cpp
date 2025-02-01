#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class ParticleFilterLocalization : public rclcpp::Node
{
  public:
    ParticleFilterLocalization()
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

  private:
    void loop()
    {
      this->publish_pose();
    }
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      scan_ = msg;
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      odom_ = msg;
    }
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
      map_ = msg;
    }
    void publish_pose()
    {
      auto message = geometry_msgs::msg::PoseStamped();
      pose_publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    nav_msgs::msg::Odometry::SharedPtr odom_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    geometry_msgs::msg::PoseStamped::SharedPtr pose_;
    int rate_hz_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParticleFilterLocalization>());
  rclcpp::shutdown();
  return 0;
}