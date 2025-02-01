#include "rclcpp/rclcpp.hpp"
#include "pf_localization/pf_localization.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pf_localization::ParticleFilterLocalization>());
  rclcpp::shutdown();
  return 0;
}