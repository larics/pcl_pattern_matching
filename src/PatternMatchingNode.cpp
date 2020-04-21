#include "pcl_pattern_matching/PatternMatching.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pattern_matching_node");
  ros::NodeHandle nh;
  auto wallDec = std::make_shared<pcl_pattern::PatternMatching>(nh);
  ros::spin();
}