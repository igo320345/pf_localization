#ifndef PF_LOCALIZATION_UTILS_HPP_
#define PF_LOCALIZATION_UTILS_HPP_

#include <cmath>
#include <vector>
#include <algorithm>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "Eigen/Dense"

namespace pf_localization
{
    Eigen::Matrix4d transformToMatrix(const geometry_msgs::msg::TransformStamped &transform);
    Eigen::Matrix4d poseToMatrix(const geometry_msgs::msg::Pose& pose);
    geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4d& P);
    double angleDiff(double a, double b);
    std::vector<double> linspace(double start, double end, int num);
    Eigen::Vector3d vectorCoordAdd(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    Eigen::Vector2i metricToGridCoords(double x, double y, const nav_msgs::msg::MapMetaData& map_info);
    std::vector<std::vector<uint8_t>> convertOccupancyGrid(const nav_msgs::msg::OccupancyGrid &map);
    double mapCalcRange(const std::vector<std::vector<uint8_t>>& map, const nav_msgs::msg::MapMetaData& map_info, 
                        double x, double y, double yaw, double max_range);
    
}                   
#endif