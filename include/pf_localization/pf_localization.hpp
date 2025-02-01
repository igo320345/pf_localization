#ifndef PF_LOCALIZATION_HPP_
#define PF_LOCALIZATION_HPP_

#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pf_localization/particle_filter.hpp"

using std::placeholders::_1;

namespace pf_localization 
{
    class ParticleFilterLocalization : public rclcpp::Node
    {
    public:
        ParticleFilterLocalization();
        ~ParticleFilterLocalization();
    private:
        void loop();
        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void publish_pose();

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
}

#endif