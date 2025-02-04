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
        laser_beams_(laser_beams), laser_sigma_hit_(laser_sigma_hit), 
        laser_z_hit_(laser_z_hit), laser_z_rand_(laser_z_rand), laser_z_short_(laser_z_short), 
        laser_z_max_(laser_z_max), laser_lambda_short_(laser_lambda_short), odom_alpha1_(odom_alpha1), 
        odom_alpha2_(odom_alpha2), odom_alpha3_(odom_alpha3), odom_alpha4_(odom_alpha4), 
        prev_odom_(Eigen::Vector3d::Zero()) 
    {
        current_state_ = init_state;
        particles.resize(num_particles_, Eigen::Vector3d::Zero());
        weights_.resize(num_particles_, 0);
        this->init_filter();
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
            particles[i] = Eigen::Vector3d(dist_x(gen), dist_y(gen), dist_yaw(gen));
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
            new_particles[j] = particles[s];
        }

        particles = new_particles;
        std::fill(weights_.begin(), weights_.end(), 1.0 / num_particles_);
    }
    Eigen::Vector3d ParticleFilter::localize(const Eigen::Vector3d& odom, const std::vector<float>& ranges, const nav_msgs::msg::OccupancyGrid& map)
    {  
        if (!grid_bin_) {
            map_info_ = map.info;
            grid_bin_ = std::make_shared<std::vector<std::vector<uint8_t>>>(pf_localization::convertOccupancyGrid(map));
        }

        this->sample_motion_model_odometry(odom);
        this->beam_range_finder_model(ranges);

        double sum_weights = std::accumulate(weights_.begin(), weights_.end(), 0.0);
        if (sum_weights > 0) {
            for (double &w : weights_) {
                w /= sum_weights;
            }
        }
        double sum_sq_weights = 0.0;
        for (const double &w : weights_) {
            sum_sq_weights += std::pow(w, 2);
        }
        double n_eff = (sum_sq_weights > 0) ? (1.0 / sum_sq_weights) : 0.0;
        if (n_eff < num_particles_) {
            this->resample();
        }

        this->calc_mean_state();
        return current_state_;
    }

    void ParticleFilter::sample_motion_model_odometry(const Eigen::Vector3d& odom) 
    {
        double d_trans = std::hypot(prev_odom_(0) - odom(0), prev_odom_(1) - odom(1));

        double d_rot1 = (d_trans < 0.01) ? 0.0
                                         : pf_localization::angleDiff(std::atan2(odom(1) - prev_odom_(1), odom(0) - prev_odom_(0)), prev_odom_(2));
        double d_rot2 = pf_localization::angleDiff(odom(2) - prev_odom_(2), d_rot1);

        double d_rot1_noise = std::min(std::abs(angleDiff(d_rot1, 0.0)), std::abs(angleDiff(d_rot1, M_PI)));
        double d_rot2_noise = std::min(std::abs(angleDiff(d_rot2, 0.0)), std::abs(angleDiff(d_rot2, M_PI)));

        std::normal_distribution<double> noise_rot1(0, std::sqrt(odom_alpha1_ * d_rot1_noise * d_rot1_noise + odom_alpha2_ * d_trans * d_trans));
        std::normal_distribution<double> noise_trans(0, std::sqrt(odom_alpha3_ * d_trans * d_trans + 
                                                                   odom_alpha4_ * d_rot1_noise * d_rot1_noise + 
                                                                   odom_alpha4_ * d_rot2_noise * d_rot2_noise));
        std::normal_distribution<double> noise_rot2(0, std::sqrt(odom_alpha1_ * d_rot2_noise * d_rot2_noise + odom_alpha2_ * d_trans * d_trans));
        std::random_device rd;
        std::mt19937 gen(rd());
        for (int i = 0; i < num_particles_; i++) {
            double dhat_rot1 = angleDiff(d_rot1, noise_rot1(gen));
            double dhat_trans = d_trans - noise_trans(gen);
            double dhat_rot2 = angleDiff(d_rot2, noise_rot2(gen));


            for(auto &particle : particles) {
                particle[0] += dhat_trans * std::cos(particle[2] + dhat_rot1);
                particle[1] += dhat_trans * std::sin(particle[2] + dhat_rot1);
                particle[2] += dhat_rot1 + dhat_rot2;
            }
        }

        prev_odom_ = odom;
    }
    void ParticleFilter::beam_range_finder_model(const std::vector<float>& ranges)
    {
        std::vector<double> angles = pf_localization::linspace(laser_min_angle_, laser_max_angle_, ranges.size());

        for (int i = 0; i < num_particles_; ++i) {
            double q = 1.0;
            Eigen::Vector3d pose = pf_localization::vectorCoordAdd(laser_pose_, particles[i]);  
            int step = ranges.size() / laser_beams_;

            for (uint32_t k = 0; k < ranges.size(); k += step) {
                double r = std::fmin(ranges[k], laser_max_range_);
                double pz = 0.0;
                
                double map_range = mapCalcRange(*grid_bin_, map_info_, pose[0], pose[1], pose[2] + angles[k], laser_max_range_);
                double z = r - map_range;

                pz += laser_z_hit_ * std::exp(-(z * z) / (2 * laser_sigma_hit_ * laser_sigma_hit_));

                if (z < 0) {
                    pz += laser_z_short_ * laser_lambda_short_ * std::exp(-laser_lambda_short_ * r);
                }

                if (r == laser_max_range_) {
                    pz += laser_z_max_;
                }

                if (r < laser_max_range_) {
                    pz += laser_z_rand_ / laser_max_range_;
                }

                q += std::pow(pz, 3);
            }
            weights_[i] *= q;
        }
    }
    void ParticleFilter::calc_mean_state() 
    {
        Eigen::Vector3d mean_state = Eigen::Vector3d::Zero();
        for (int i = 0; i < num_particles_; ++i) {
            mean_state += particles[i] * weights_[i];
        }
        current_state_ = mean_state;
    }
}