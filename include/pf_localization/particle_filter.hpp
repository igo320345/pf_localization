#ifndef PF_LOCALIZATION_PARTICLE_FILTER_HPP_
#define PF_LOCALIZATION_PARTICLE_FILTER_HPP_

#include <random>

#include "pf_localization/utils.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


namespace pf_localization
{
    class ParticleFilter 
    {
        public:
            ParticleFilter(
            const Eigen::Vector3d& laser_pose,
            double laser_min_angle,
            double laser_max_angle,
            double laser_max_range,
            int num_particles,
            const Eigen::Vector3d& init_state,
            int laser_beams,
            double laser_sigma_hit,
            double laser_z_hit,
            double laser_z_rand,
            double laser_z_short,
            double laser_z_max,
            double laser_lambda_short,
            double odom_alpha1,
            double odom_alpha2,
            double odom_alpha3,
            double odom_alpha4);
            ~ParticleFilter();

            void init_filter();
            void resample();
            Eigen::Vector3d localize(const Eigen::Vector3d& odom, const std::vector<float>& ranges, const nav_msgs::msg::OccupancyGrid& map);
        private:
            void sample_motion_model_odometry(const Eigen::Vector3d& odom);
            void beam_range_finder_model(const std::vector<float>& ranges);
            void calc_mean_state();

            
            Eigen::Vector3d laser_pose_;
            double laser_min_angle_, laser_max_angle_, laser_max_range_;
            int num_particles_;
            Eigen::Vector3d init_state_;
            int laser_beams_;
            double laser_sigma_hit_, laser_z_hit_, laser_z_rand_, laser_z_short_, laser_z_max_, laser_lambda_short_;
            double odom_alpha1_, odom_alpha2_, odom_alpha3_, odom_alpha4_;
            Eigen::Vector3d current_state_;
            Eigen::Vector3d prev_odom_;
            std::vector<Eigen::Vector3d> particles_;
            std::vector<double> weights_;
            std::shared_ptr<std::vector<std::vector<uint8_t>>> grid_bin_;
            nav_msgs::msg::MapMetaData map_info_;
    };
}
#endif