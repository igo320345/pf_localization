#include "rclcpp/rclcpp.hpp"
#include "pf_localization/pf_localization.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pf_localization::ParticleFilterLocalization>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}