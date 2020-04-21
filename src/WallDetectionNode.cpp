#include "pcl_pattern_matching/WallDetection.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wall_detection");
  ros::NodeHandle nh;
  auto wallDec = std::make_shared<detection::WallDetection>(nh);
  ros::spin();
}