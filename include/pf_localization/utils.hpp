#ifndef PF_LOCALIZATION_UTILS_HPP_
#define PF_LOCALIZATION_UTILS_HPP_

#include <cmath>
#include <vector>
#include <algorithm>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "Eigen/Dense"

Eigen::Matrix4d transformToMatrix(const std::pair<Eigen::Vector3d, tf2::Quaternion>& transform);
Eigen::Matrix4d poseToMatrix(const geometry_msgs::msg::Pose& pose);
geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4d& P);
double angleDiff(double a, double b);
Eigen::Vector3d vectorCoordAdd(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
Eigen::Vector2i metricToGridCoords(double x, double y, const nav_msgs::msg::MapMetaData& map_info);
double mapCalcRange(const std::vector<std::vector<int>>& map, const nav_msgs::msg::MapMetaData& map_info, 
                    double x, double y, double yaw, double max_range);
                    
#endif