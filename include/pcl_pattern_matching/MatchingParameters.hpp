#ifndef MATCHING_PARAMETERS_HPP
#define MATCHING_PARAMETERS_HPP

#include <limits>

namespace pcl_params {

/**
 * @brief Initial parameters for 3D model pattern matching in the pointcloud.
 *
 */
struct MatchingParams
{

  // Crop box dimensions
  static constexpr auto CROP_BOX_DIM = std::numeric_limits<double>::max();
  static constexpr auto MIN_CROP_HEIGHT = 2.;
  static constexpr auto MAX_CROP_HEIGHT = 2.5;

  // Parameters of the outlier filters
  static constexpr auto OUTLIER_FILTER_MEAN = 100;
  static constexpr auto OUTLIER_FILTER_STDDEV = 0.1;

  // Upsample parameters for the 3D model pattern
  static constexpr auto UPSAMPLE_INCREMENT = 0.01;
  static constexpr auto UPSAMPLE_LIMIT = 0.75;
  static constexpr auto UPSAMPLE_OFFSET = 0.01;
  static constexpr auto UPSAMPLE_ITER =
    static_cast<int>(MatchingParams::UPSAMPLE_LIMIT / MatchingParams::UPSAMPLE_INCREMENT);

  // Dilation factor for the pointcloud horizontal image
  static constexpr auto DILATION_FACTOR = 9;

  // Organized pointcloud parameters
  static constexpr auto ORG_PCL_RESOLUTION = 20.0;
  static constexpr auto ORG_PCL_WIDTH = 100;
  static constexpr auto ORG_PCL_HEIGHT = 100;

  // Minimum point count to be considered as a valid detection
  static constexpr auto MIN_POINT_COUNT = 3500;

  // Threshold in [m] for the detection to be considered to have the same pose
  static constexpr auto MAX_POSE_DISTANCE_TOLERANCE = 3;

  // Threshold for repeating same pose count
  static constexpr auto POSE_COUNT_THRESHOLD = 2;

  // Time between detection loop [s]
  static constexpr auto LOOP_TIME = 0.1;

  // Pointcloud scaling factors
  static constexpr auto SCALING_FACTOR = 1000.0;

  // Warning message timeout
  static constexpr auto WARN_TIME = 5;
};
}// namespace pcl_params

#endif