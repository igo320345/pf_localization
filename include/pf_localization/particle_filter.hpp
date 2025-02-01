#ifndef PF_LOCALIZATION_PARTICLE_FILTER_HPP_
#define PF_LOCALIZATION_PARTICLE_FILTER_HPP_

#include "pf_localization/utils.hpp"

namespace pf_localization
{
    class ParticleFilter 
    {
        public:
            ParticleFilter(
            int num_particles, 
            int laser_beams,
            float laser_sigma_hit, 
            float laser_z_hit,
            float laser_z_rand,
            float laser_z_short,
            float laser_z_max,
            float laser_lambda_short,
            float odom_alpha1,
            float odom_alpha2,
            float odom_alpha3,
            float odom_alpha4);
            ~ParticleFilter();

            void init_filter();
            void resample();
            void localize();
        private:
            void sample_motion_model_odometry();
            void beam_range_finder_model();
            void calc_mean_state();
    };
}
#endif