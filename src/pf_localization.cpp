#include "pf_localization/pf_localization.hpp"

namespace pf_localization
{
  ParticleFilterLocalization::ParticleFilterLocalization()
  : Node("pf_localization_node"), rate_hz_(30)
  {
    this->declare_parameter("base_frame_id", "diff_drive/base_link");
    this->declare_parameter("odom_frame_id", "diff_drive/odom");
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
    particles_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("particles", 10);
    laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "diff_drive/scan", 10, std::bind(&ParticleFilterLocalization::laserCallback, this, _1));
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "diff_drive/odometry", 10, std::bind(&ParticleFilterLocalization::odomCallback, this, _1));
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", 10, std::bind(&ParticleFilterLocalization::mapCallback, this, _1));
  }

  ParticleFilterLocalization::~ParticleFilterLocalization()
  {
    
  }

  void ParticleFilterLocalization::spin() 
  {
    rclcpp::Rate rate(rate_hz_);
    while (rclcpp::ok()) {
        if (odom_ && scan_ && map_) {
            if (!particle_filter_) {
                this->createParticleFilter();
            } else {
                this->publishPose();
                this->publishParticles();
            }
        }
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
  }
  void ParticleFilterLocalization::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_ = msg;
  }
  void ParticleFilterLocalization::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = msg;
  }
  void ParticleFilterLocalization::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
  }
  void ParticleFilterLocalization::publishPose()
  {
    auto odom = Eigen::Vector3d(odom_->pose.pose.position.x, 
    odom_->pose.pose.position.y,
    tf2::getYaw(odom_->pose.pose.orientation));
    auto pose = particle_filter_->localize(odom, scan_->ranges, *map_);
    pose_->header.stamp = this->get_clock()->now();
    pose_->pose.position.x = pose(0);
    pose_->pose.position.y = pose(1);
    tf2::Quaternion q;
    q.setRPY(0, 0, pose(2));
    pose_->pose.orientation.x = q.x();
    pose_->pose.orientation.y = q.y();
    pose_->pose.orientation.z = q.z();
    pose_->pose.orientation.w = q.w();
    pose_publisher_->publish(*pose_);

    auto t = tf_buffer_->lookupTransform(odom_frame_id_, base_frame_id_, tf2::TimePointZero);
    auto matrix_t = pf_localization::transformToMatrix(t);
    auto matrix_pose = pf_localization::poseToMatrix(pose_->pose);
    auto matrix_map_transform = matrix_pose * matrix_t.inverse();
    auto map_transform = pf_localization::matrixToPose(matrix_map_transform);
    auto transform_msg = geometry_msgs::msg::TransformStamped();
    transform_msg.header.stamp = this->get_clock()->now();
    transform_msg.header.frame_id = global_frame_id_;
    transform_msg.child_frame_id = odom_frame_id_;
    transform_msg.transform.translation.x = map_transform.position.x;
    transform_msg.transform.translation.y = map_transform.position.y;
    transform_msg.transform.translation.z = map_transform.position.z;
    transform_msg.transform.rotation = map_transform.orientation;
    tf_broadcaster_->sendTransform(transform_msg);
  }
  void ParticleFilterLocalization::publishParticles()
  {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto t = this->get_clock()->now();
    int i = 0;
    for (auto &particle : this->particle_filter_->particles_) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.stamp = t;
      marker.header.frame_id = global_frame_id_;
      marker.ns = "particles";
      marker.id = i;
      marker.type = 0;
      marker.action = 0;
      marker.lifetime = rclcpp::Duration::from_seconds(1.0);
      double yaw_in_map = particle[2];
      double vx = std::cos(yaw_in_map);
      double vy = std::sin(yaw_in_map);
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      auto point1 = geometry_msgs::msg::Point();
      auto point2 = geometry_msgs::msg::Point();
      point1.x = particle[0];
      point1.y = particle[1];
      point1.z = 0.2;
      point2.x = particle[0] + 0.3 * vx;
      point2.y = particle[1] + 0.3 * vy;
      point2.z = 0.2;
      marker.points.push_back(point1);
      marker.points.push_back(point2);
      marker.scale.x = 0.05;
      marker.scale.y = 0.15;
      marker.scale.z = 0.1;
      marker_array.markers.push_back(marker);
      i++;
    }
    particles_publisher_->publish(marker_array);
  }
  void ParticleFilterLocalization::createParticleFilter()
  {
    try {
      auto t = tf_buffer_->lookupTransform(base_frame_id_, scan_->header.frame_id, tf2::TimePointZero);
      laser_pose_ = Eigen::Vector3d(t.transform.translation.x, t.transform.translation.y, tf2::getYaw(t.transform.rotation));
      laser_min_angle_ = scan_->angle_min;
      laser_max_angle_ = scan_->angle_max;
      laser_max_range_= scan_->range_max;
      init_state_ = Eigen::Vector3d::Zero();
      particle_filter_ = std::make_shared<pf_localization::ParticleFilter>(laser_pose_, laser_min_angle_,
      laser_max_angle_, laser_max_range_, num_particles_, init_state_, laser_beams_, laser_sigma_hit_, 
      laser_z_hit_, laser_z_rand_, laser_z_short_, laser_z_max_, laser_lambda_short_, odom_alpha1_, 
      odom_alpha2_, odom_alpha3_, odom_alpha4_);
      RCLCPP_INFO(this->get_logger(), "pf created!");
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }
}