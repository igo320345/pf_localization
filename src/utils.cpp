#include "pf_localization/utils.hpp"

namespace pf_localization
{
    Eigen::Matrix4d transformToMatrix(const geometry_msgs::msg::TransformStamped &transform) {
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

        matrix(0, 3) = transform.transform.translation.x;
        matrix(1, 3) = transform.transform.translation.y;
        matrix(2, 3) = transform.transform.translation.z;

        tf2::Quaternion q(transform.transform.rotation.x, 
                        transform.transform.rotation.y, 
                        transform.transform.rotation.z, 
                        transform.transform.rotation.w);
        tf2::Matrix3x3 rotation_matrix(q);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                matrix(i, j) = rotation_matrix[i][j];
            }
        }

        return matrix;
    }

    Eigen::Matrix4d poseToMatrix(const geometry_msgs::msg::Pose& pose) {
        Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
        
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 rotationMatrix(q);

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                P(i, j) = rotationMatrix[i][j];

        P(0, 3) = pose.position.x;
        P(1, 3) = pose.position.y;
        P(2, 3) = pose.position.z;

        return P;
    }

    geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4d& P) {
        geometry_msgs::msg::Pose pose;

        pose.position.x = P(0, 3);
        pose.position.y = P(1, 3);
        pose.position.z = P(2, 3);

        tf2::Matrix3x3 rotationMatrix(
            P(0, 0), P(0, 1), P(0, 2),
            P(1, 0), P(1, 1), P(1, 2),
            P(2, 0), P(2, 1), P(2, 2)
        );

        tf2::Quaternion q;
        rotationMatrix.getRotation(q);

        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        return pose;
    }

    double angleDiff(double a, double b) {
        a = std::atan2(std::sin(a), std::cos(a));
        b = std::atan2(std::sin(b), std::cos(b));

        double d1 = a - b;
        double d2 = 2 * M_PI - std::abs(d1);

        if (d1 > 0.0) {
            d2 *= -1.0;
        }

        return (std::abs(d1) < std::abs(d2)) ? d1 : d2;
    }

    std::vector<double> linspace(double start, double end, int num) {
        std::vector<double> result(num);
        double step = (end - start) / (num - 1);
        for (int i = 0; i < num; ++i) {
            result[i] = start + i * step;
        }
        return result;
    }

    Eigen::Vector3d vectorCoordAdd(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        Eigen::Vector3d c;
        c[0] = b[0] + a[0] * std::cos(b[2]) - a[1] * std::sin(b[2]);
        c[1] = b[1] + a[0] * std::sin(b[2]) + a[1] * std::cos(b[2]);
        c[2] = b[2] + a[2];

        c[2] = std::atan2(std::sin(c[2]), std::cos(c[2]));

        return c;
    }

    Eigen::Vector2i metricToGridCoords(double x, double y, const nav_msgs::msg::MapMetaData& map_info) {
        int gx = static_cast<int>((x - map_info.origin.position.x) / map_info.resolution);
        int gy = static_cast<int>((y - map_info.origin.position.y) / map_info.resolution);
        return Eigen::Vector2i(std::max(0, std::min(gx, static_cast<int>(map_info.width))),
                                std::max(0, std::min(gy, static_cast<int>(map_info.height))));
    }

    std::vector<std::vector<uint8_t>> convertOccupancyGrid(const nav_msgs::msg::OccupancyGrid &map) {
        int height = map.info.height;
        int width = map.info.width;

        std::vector<std::vector<uint8_t>> grid_bin(height, std::vector<uint8_t>(width, 0));

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                grid_bin[y][x] = (map.data[index] == 0) ? 1 : 0;
            }
        }

        return grid_bin;
    }

    double mapCalcRange(const std::vector<std::vector<uint8_t>>& map, const nav_msgs::msg::MapMetaData& map_info, 
                        double x, double y, double yaw, double max_range) {
        
        auto xy0 = metricToGridCoords(x, y, map_info);
        auto xy1 = metricToGridCoords(x + max_range * std::cos(yaw), y + max_range * std::sin(yaw), map_info);

        int x0 = xy0[0], y0 = xy0[1];
        int x1 = xy1[0], y1 = xy1[1];

        bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);

        if (steep) {
            std::swap(x0, y0);
            std::swap(x1, y1);
        }

        int deltax = std::abs(x1 - x0);
        int deltay = std::abs(y1 - y0);
        int error = 0;
        int deltaerr = deltay;

        x = x0;
        y = y0;
        
        int xstep = (x0 < x1) ? 1 : -1;
        int ystep = (y0 < y1) ? 1 : -1;

        auto isOutOfBounds = [&](int x, int y) {
            return x < 0 || y < 0 || x >= static_cast<int>(map[0].size()) || y >= static_cast<int>(map.size());
        };

        auto isObstacle = [&](int x, int y) {
            return isOutOfBounds(x, y) || map[y][x] == 0; 
        };

        if (steep) {
            if (isObstacle(y, x)) {
                return std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map_info.resolution;
            }
        } else {
            if (isObstacle(x, y)) {
                return std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map_info.resolution;
            }
        }

        while (x != x1 + xstep) {
            x += xstep;
            error += deltaerr;

            if (2 * error >= deltax) {
                y += ystep;
                error -= deltax;
            }

            if (steep) {
                if (isObstacle(y, x)) {
                    return std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map_info.resolution;
                }
            } else {
                if (isObstacle(x, y)) {
                    return std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map_info.resolution;
                }
            }
        }

        return max_range;
    }
}