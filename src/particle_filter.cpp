#include "pf_localization/particle_filter.hpp"

namespace pf_localization
{
    ParticleFilter::ParticleFilter(
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
    double odom_alpha4
    ) : laser_pose_(laser_pose), laser_min_angle_(laser_min_angle), laser_max_angle_(laser_max_angle),
        laser_max_range_(laser_max_range), num_particles_(num_particles), init_state_(init_state),
        current_state_(init_state), laser_beams_(laser_beams), laser_sigma_hit_(laser_sigma_hit), 
        laser_z_hit_(laser_z_hit), laser_z_rand_(laser_z_rand), laser_z_short_(laser_z_short), 
        laser_z_max_(laser_z_max), laser_lambda_short_(laser_lambda_short), odom_alpha1_(odom_alpha1), 
        odom_alpha2_(odom_alpha2), odom_alpha3_(odom_alpha3), odom_alpha4_(odom_alpha4), 
        prev_odom_(Eigen::Vector3d::Zero()) 
    {
        particles_.resize(num_particles_, Eigen::Vector3d::Zero());
        weights_.resize(num_particles_, 0);
        init_filter();
    }
    ParticleFilter::~ParticleFilter() 
    {

    }
    void ParticleFilter::init_filter() 
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> dist_x(init_state_(0), 0.1);
        std::normal_distribution<double> dist_y(init_state_(1), 0.1);
        std::normal_distribution<double> dist_yaw(init_state_(2), M_PI / 4);

        for (int i = 0; i < num_particles_; ++i) {
            particles_[i] = Eigen::Vector3d(dist_x(gen), dist_y(gen), dist_yaw(gen));
        }
        weights_.resize(num_particles_, 1.0 / num_particles_);
    }
    void ParticleFilter::resample()
    {
        std::vector<double> cValues(num_particles_);
        cValues[0] = weights_[0];
        for (int i = 1; i < num_particles_; ++i) {
            cValues[i] = cValues[i - 1] + weights_[i];
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
        double startingPoint = dist(gen);
        std::vector<Eigen::Vector3d> new_particles(num_particles_);

        for (int j = 0; j < num_particles_; ++j) {
            double currentPoint = startingPoint + (1.0 / num_particles_) * j;
            int s = 0;
            while (currentPoint > cValues[s]) {
                s++;
            }
            new_particles[j] = particles_[s];
        }

        particles_ = new_particles;
        std::fill(weights_.begin(), weights_.end(), 1.0 / num_particles_);
    }
    Eigen::Vector3d ParticleFilter::localize(const Eigen::Vector3d& odom, const std::vector<double>& ranges, const nav_msgs::msg::MapMetaData& map)
    {
        // TODO
    }

    void ParticleFilter::sample_motion_model_odometry(const Eigen::Vector3d& odom) 
    {
        // TODO
    }
    void ParticleFilter::beam_range_finder_model(const std::vector<double>& ranges)
    {
        // TODO
    }
    void ParticleFilter::calc_mean_state() 
    {
        Eigen::Vector3d mean_state = Eigen::Vector3d::Zero();
        for (int i = 0; i < num_particles_; ++i) {
            mean_state += particles_[i] * weights_[i];
        }
        current_state_ = mean_state;
    }
}