#ifndef WALL_DETECTION_H
#define WALL_DETECTION_H

#include <vector>

#include "pcl_pattern_matching/Global2Local.hpp"
#include "pcl_pattern_matching/MatchingParameters.hpp"
#include "pcl_pattern_matching/PCLUtil.hpp"
#include "pcl_pattern_matching/Util.hpp"
#include <pcl_pattern_matching/PatternMatchingParametersConfig.h>

#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>

#include <compressed_image_transport/compressed_publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

using namespace ros_util;
using namespace pcl_params;
using PCXYZ = pcl::PointCloud<pcl::PointXYZ>;
using ROSCloud = sensor_msgs::PointCloud2;
using DetectionConfig = pcl_pattern_matching::PatternMatchingParametersConfig;

namespace pcl_pattern {

class PatternMatching
{
public:
  explicit PatternMatching(ros::NodeHandle &t_nh)
    : m_handlerMapCloud(t_nh, "submap_cloud"), m_g2l(t_nh), m_nh(t_nh)
  {
    initialize_parameters(t_nh);
    m_pubFilteredCloud = t_nh.advertise<ROSCloud>("filtered_cloud", 1);
    m_pubTargetCloud = t_nh.advertise<ROSCloud>("target_cloud", 1);
    m_pubWallOdometry = t_nh.advertise<nav_msgs::Odometry>("wall/odometry", 1);

    image_transport::ImageTransport it(t_nh);
    m_pubPointCloudImage = it.advertise("cloud_image", 1);
    m_pubResultImage = it.advertise("result_image", 1);
    m_pubTargetImage = it.advertise("target_image", 1);

    m_loopTimer =
      t_nh.createTimer(MatchingParams::LOOP_TIME, &PatternMatching::loop_event, this);

    m_targetCloud = target_from_ply("resources/wall_pattern_upscaled.ply");
    m_targetCloud = pcl_util::organize_pointcloud(m_targetCloud,
      MatchingParams::ORG_PCL_RESOLUTION,
      MatchingParams::ORG_PCL_WIDTH,
      MatchingParams::ORG_PCL_HEIGHT);
    m_targetImage = pcl_util::PCXYZ_to_cvMat(m_targetCloud);
  }

private:
  PCXYZ::Ptr target_from_ply(const std::string &plyFilename)
  {
    const auto plyPath =
      ros::package::getPath("pcl_pattern_matching") + "/" + plyFilename;
    auto wallCloud = pcl_util::pcl_from_ply(plyPath);

    auto wallWithMean = pcl_util::upsample_pointcloud(wallCloud,
      MatchingParams::SCALING_FACTOR,
      MatchingParams::UPSAMPLE_INCREMENT,
      MatchingParams::UPSAMPLE_OFFSET,
      MatchingParams::UPSAMPLE_ITER);
    // Final wall cloud is the demeaned cloud
    Eigen::Vector4f targetCentroid;
    pcl::compute3DCentroid(*wallWithMean, targetCentroid);
    wallCloud = pcl_util::demean_pointcloud(wallWithMean, targetCentroid);
    return wallCloud;
  }

  Eigen::Matrix4f template_matching(const cv::Mat &t_source8UC1,
    const cv::Mat &t_target8UC1)
  {
    if (t_source8UC1.empty()) {
      ROS_WARN_THROTTLE(
        MatchingParams::WARN_TIME, "template_matching - source is empty.");
      return {};
    }

    // Dilate the pointcloud
    cv::Mat resultImage(t_source8UC1);
    double dilation_size = m_handlerParam->getData().dilation_factor;
    cv::Mat element = getStructuringElement(cv::MORPH_RECT,
      cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
      cv::Point(dilation_size, dilation_size));
    cv::dilate(resultImage, resultImage, element);

    using ContourVector = std::vector<std::vector<cv::Point>>;
    ContourVector sourceContours;
    ContourVector targetContours;
    cv::findContours(resultImage, sourceContours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    cv::findContours(t_target8UC1, targetContours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    ROS_INFO("Source contour count: %lu", sourceContours.size());
    ROS_INFO("Target contour count: %lu", targetContours.size());

    int bestMatchedIndex = -1;
    static constexpr auto INITIAL_FITNESS = 1e5;
    double bestFitness = INITIAL_FITNESS;
    for (int i = 0; i < sourceContours.size(); i++) {

      if (cv::contourArea(sourceContours[i])
          < m_handlerParam->getData().min_point_count) {
        continue;
      }

      double currentFitness = cv::matchShapes(
        sourceContours[i], targetContours.front(), CV_CONTOURS_MATCH_I3, 0);
      if (currentFitness < bestFitness) {
        bestMatchedIndex = i;
        bestFitness = currentFitness;
      }
    }

    if (bestMatchedIndex < 0) {
      ROS_FATAL("PatternMatching::template_matching - unable to match contours");
      return {};
    }

    ROS_INFO("Best fitness: %.5f", bestFitness);
    static constexpr auto DRAW_COLOR = 150;
    static constexpr auto DRAW_THICKNESS = 5;
    cv::drawContours(resultImage,
      sourceContours,
      bestMatchedIndex,
      cv::Scalar(DRAW_COLOR),
      DRAW_THICKNESS);
    cv::Moments M = cv::moments(sourceContours[bestMatchedIndex]);
    int cX = int(M.m10 / M.m00);
    int cY = int(M.m01 / M.m00);
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(1, 3) = (cX - t_source8UC1.rows / 2.0) / MatchingParams::ORG_PCL_RESOLUTION
                      + m_lastFoundMapCentroid.y();
    transform(0, 3) = (cY - t_source8UC1.cols / 2.0) / MatchingParams::ORG_PCL_RESOLUTION
                      + m_lastFoundMapCentroid.x();
    transform(2, 3) = m_lastFoundMapCentroid.z();
    ROS_INFO_STREAM("Target transform: " << transform);

    double distance_from_previous =
      sqrt(pow(transform(0, 3) - m_bestTransformation(0, 3), 2)
           + pow(transform(1, 3) - m_bestTransformation(1, 3), 2));

    if (distance_from_previous > MatchingParams::MAX_POSE_DISTANCE_TOLERANCE) {
      ROS_WARN("PatternMatching - resetting detection counter.");
      m_detectionCounter = 0;
    } else {
      m_detectionCounter++;
      ROS_INFO("PatternMatching - counter at %d", m_detectionCounter);
    }
    m_bestTransformation = transform;

    publish_image(resultImage, m_pubResultImage);
    publish_image(t_target8UC1, m_pubTargetImage);
    publish_wall_odometry(m_bestTransformation);

    if (m_detectionCounter >= MatchingParams::POSE_COUNT_THRESHOLD) {
      auto wallGlobal =
        m_g2l.toGlobal(m_bestTransformation(0, 3), m_bestTransformation(1, 3), 0);
      m_nh.setParam("brick_dropoff/lat", wallGlobal.x());
      m_nh.setParam("brick_dropoff/lon", wallGlobal.y());
      ROS_WARN("Set new pickup position! [%.10f, %.10f]", wallGlobal.x(), wallGlobal.y());
    }
    return transform;
  }

  void publish_wall_odometry(const Eigen::Matrix4f &t_transform)
  {
    const auto get_translation_x = [&t_transform]() { return t_transform(0, 3); };
    const auto get_translation_y = [&t_transform]() { return t_transform(1, 3); };
    const auto get_translation_z = [&t_transform]() { return t_transform(2, 3); };
    const auto get_transform_quaternion = [&t_transform]() {
      tf::Matrix3x3 mat(t_transform(0, 0),
        t_transform(0, 1),
        t_transform(0, 2),
        t_transform(1, 0),
        t_transform(1, 1),
        t_transform(1, 2),
        t_transform(2, 0),
        t_transform(2, 1),
        t_transform(2, 2));
      double roll;
      double pitch;
      double yaw;
      mat.getRPY(roll, pitch, yaw);
      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      return q;
    };

    nav_msgs::Odometry wallOdom;
    wallOdom.header.stamp = ros::Time::now();
    wallOdom.header.frame_id = ros::this_node::getNamespace() + "/map";
    wallOdom.pose.pose.position.x = get_translation_x();
    wallOdom.pose.pose.position.y = get_translation_y();
    wallOdom.pose.pose.position.z = get_translation_z();

    auto q = get_transform_quaternion();
    wallOdom.pose.pose.orientation.x = q.getX();
    wallOdom.pose.pose.orientation.y = q.getY();
    wallOdom.pose.pose.orientation.z = q.getZ();
    wallOdom.pose.pose.orientation.w = q.getW();

    m_pubWallOdometry.publish(wallOdom);
  }

  void initialize_parameters(ros::NodeHandle & /*unused*/)
  {
    DetectionConfig config;
    config.outlier_filter_mean = MatchingParams::OUTLIER_FILTER_MEAN;
    config.outlier_filter_stddev = MatchingParams::OUTLIER_FILTER_STDDEV;
    config.min_crop_height = MatchingParams::MIN_CROP_HEIGHT;
    config.max_crop_height = MatchingParams::MAX_CROP_HEIGHT;
    config.dilation_factor = MatchingParams::DILATION_FACTOR;
    config.min_point_count = MatchingParams::MIN_POINT_COUNT;
    m_handlerParam =
      std::make_shared<ParamHandler<DetectionConfig>>(config, "wall_detection");
  }

  void loop_event(const ros::TimerEvent & /*unused*/)
  {
    // Convert from ROS message to pcl
    auto inputCloud = boost::make_shared<PCXYZ>();
    read_input_cloud(inputCloud);
    inputCloud = pcl_util::box_filter(inputCloud,
      -MatchingParams::CROP_BOX_DIM,
      MatchingParams::CROP_BOX_DIM,
      -MatchingParams::CROP_BOX_DIM,
      MatchingParams::CROP_BOX_DIM,
      m_handlerParam->getData().min_crop_height,
      m_handlerParam->getData().max_crop_height);
    inputCloud = pcl_util::do_outlier_filtering(inputCloud,
      m_handlerParam->getData().outlier_filter_mean,
      m_handlerParam->getData().outlier_filter_stddev);
    pcl::compute3DCentroid(*inputCloud, m_lastFoundMapCentroid);
    inputCloud = pcl_util::demean_pointcloud(inputCloud, m_lastFoundMapCentroid);
    inputCloud = pcl_util::organize_pointcloud(inputCloud,
      MatchingParams::ORG_PCL_RESOLUTION,
      MatchingParams::ORG_PCL_WIDTH,
      MatchingParams::ORG_PCL_HEIGHT);
    auto inputCloudImage = pcl_util::PCXYZ_to_cvMat(inputCloud);
    auto targetTransform = template_matching(inputCloudImage, m_targetImage);

    // auto transformedTargetCloud = boost::make_shared<PCXYZ>();
    // pcl::transformPointCloud(*m_targetCloud, *transformedTargetCloud,
    // targetTransform); do_icp(inputCloud, transformedTargetCloud); // TODO:
    // Try dilating for ICP aswell

    publish_image(inputCloudImage, m_pubPointCloudImage);
    publish_cloud(inputCloud, m_pubFilteredCloud);
    publish_cloud(m_targetCloud, m_pubTargetCloud);
  }

  void publish_image(const cv::Mat &t_image, image_transport::Publisher &t_pub)
  {
    cv_bridge::CvImage bridgeImage;
    bridgeImage.header.frame_id = ros::this_node::getNamespace() + "/map";
    bridgeImage.header.stamp = ros::Time::now();
    bridgeImage.header.seq = m_handlerMapCloud.getData().header.seq;
    bridgeImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    bridgeImage.image = t_image;
    t_pub.publish(bridgeImage.toImageMsg());
  }

  inline void read_input_cloud(PCXYZ::Ptr &t_inputCloud)
  {
    pcl::fromROSMsg(m_handlerMapCloud.getData(), *t_inputCloud);
  }

  static void publish_cloud(const PCXYZ::Ptr &t_inputCloud, ros::Publisher &t_pub)
  {
    auto rosMsg = boost::make_shared<ROSCloud>();
    pcl::toROSMsg(*t_inputCloud, *rosMsg);
    rosMsg->header.frame_id = ros::this_node::getNamespace() + "/map";
    rosMsg->header.stamp = ros::Time::now();
    t_pub.publish(*rosMsg);
  }

  ros::Timer m_loopTimer;
  Eigen::Vector4f m_lastFoundMapCentroid;
  Eigen::Matrix4f m_bestTransformation;

  ros::Publisher m_pubFilteredCloud, m_pubTargetCloud, m_pubWallOdometry;
  image_transport::Publisher m_pubPointCloudImage, m_pubResultImage, m_pubTargetImage;

  TopicHandler<ROSCloud> m_handlerMapCloud;
  std::shared_ptr<ParamHandler<DetectionConfig>> m_handlerParam;
  PCXYZ::Ptr m_targetCloud;
  cv::Mat m_targetImage;

  Global2Local m_g2l;
  ros::NodeHandle m_nh;
  int m_detectionCounter = 0;
};

}// namespace pcl_pattern
#endif /* WALL_DETECTION_H */