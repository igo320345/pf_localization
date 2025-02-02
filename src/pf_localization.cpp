#include "pf_localization/pf_localization.hpp"

namespace pf_localization
{
  ParticleFilterLocalization::ParticleFilterLocalization()
  : Node("pf_localization_node"), rate_hz_(30)
  {
    this->declare_parameter("base_frame_id", "base_link");
    this->declare_parameter("odom_frame_id", "odom");
    this->declare_parameter("global_frame_id", "map");
    this->declare_parameter("num_particles", 100);
    this->declare_parameter("laser_beams", 8);
    this->declare_parameter("laser_sigma_hit", 0.2);
    this->declare_parameter("laser_z_hit", 0.95);
    this->declare_parameter("laser_z_rand", 0.05);
    this->declare_parameter("laser_z_short", 0.1);
    this->declare_parameter("laser_z_max", 0.05);
    this->declare_parameter("laser_lambda_short", 0.1);
    this->declare_parameter("odom_alpha1", 0.2);
    this->declare_parameter("odom_alpha2", 0.2);
    this->declare_parameter("odom_alpha3", 0.2);
    this->declare_parameter("odom_alpha4", 0.2);

    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    global_frame_id_ = this->get_parameter("global_frame_id").as_string();
    num_particles_ = this->get_parameter("num_particles").as_int();
    laser_beams_ = this->get_parameter("laser_beams").as_int();
    laser_sigma_hit_ = this->get_parameter("laser_sigma_hit").as_double();
    laser_z_hit_ = this->get_parameter("laser_z_hit").as_double();
    laser_z_rand_ = this->get_parameter("laser_z_rand").as_double();
    laser_z_short_ = this->get_parameter("laser_z_short").as_double();
    laser_z_max_ = this->get_parameter("laser_z_max").as_double();
    laser_lambda_short_ = this->get_parameter("laser_lambda_short").as_double();
    odom_alpha1_ = this->get_parameter("odom_alpha1").as_double();
    odom_alpha2_ = this->get_parameter("odom_alpha2").as_double();
    odom_alpha3_ = this->get_parameter("odom_alpha3").as_double();
    odom_alpha4_ = this->get_parameter("odom_alpha4").as_double();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose_->header.stamp = this->get_clock()->now();
    pose_->header.frame_id = global_frame_id_;

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
    if (odom_ != nullptr && map_ != nullptr && scan_ != nullptr) {
      if (particle_filter_ == nullptr) {
        create_pf();
      }
      else {
        this->publish_pose();
      }
    }
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
  void ParticleFilterLocalization::create_pf()
  {
    auto t = tf_buffer_->lookupTransform(base_frame_id_, scan_->header.frame_id, tf2::TimePointZero);
    laser_pose_[0] = t.transform.translation.x;
    laser_pose_[1] = t.transform.translation.y;
    laser_pose_[2] = 0; // TODO
    laser_min_angle_ = scan_->angle_min;
    laser_max_angle_ = scan_->angle_max;
    laser_max_range_= scan_->range_max;
    init_state_ = Eigen::Vector3d::Zero();

    particle_filter_ = std::make_shared<pf_localization::ParticleFilter>(laser_pose_, laser_min_angle_,
    laser_max_angle_, laser_max_range_, num_particles_, init_state_, laser_beams_, laser_sigma_hit_, 
    laser_z_hit_, laser_z_rand_, laser_z_short_, laser_z_max_, laser_lambda_short_, odom_alpha1_, 
    odom_alpha2_, odom_alpha3_, odom_alpha4_);
  }
}