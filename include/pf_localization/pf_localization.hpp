#ifndef PF_LOCALIZATION_HPP_
#define PF_LOCALIZATION_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
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
        void create_pf();

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
        sensor_msgs::msg::LaserScan::SharedPtr scan_;
        nav_msgs::msg::Odometry::SharedPtr odom_;
        nav_msgs::msg::OccupancyGrid::SharedPtr map_;
        geometry_msgs::msg::PoseStamped::SharedPtr pose_;
        std::shared_ptr<pf_localization::ParticleFilter> particle_filter_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        Eigen::Vector3d laser_pose_;
        Eigen::Vector3d init_state_;
        int rate_hz_;
        std::string base_frame_id_;
        std::string odom_frame_id_;
        std::string global_frame_id_;
        int num_particles_;
        int laser_beams_;
        double laser_sigma_hit_, laser_z_hit_, laser_z_rand_, laser_z_short_, laser_z_max_, laser_lambda_short_;
        double laser_min_angle_, laser_max_angle_, laser_max_range_;
        double odom_alpha1_, odom_alpha2_, odom_alpha3_, odom_alpha4_;
    };
}

#endif