#ifndef WALL_DETECTION_H
#define WALL_DETECTION_H

#include "pcl_pattern_matching/Global2Local.hpp"
#include "pcl_pattern_matching/Util.hpp"
#include <algorithm>
#include <limits>
#include <nav_msgs/Odometry.h>
#include <pcl_pattern_matching/WallDetectionParametersConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

#include "pcl/common/impl/centroid.hpp"
#include <boost/make_shared.hpp>
#include <pcl/PointIndices.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// Include CvBridge, Image Transport, Image msg
#include <compressed_image_transport/compressed_publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

namespace detection {

struct WallDetectionParameters {
  static constexpr double MAX_BOX = std::numeric_limits<double>::max();
  static constexpr double MIN_HEIGHT = 2;
  static constexpr double MAX_HEIGHT = 2.5;
  static constexpr double OUTLIER_MEAN = 100;
  static constexpr double OUTLIER_STDDEV = 0.1;
  static constexpr float UPSCALE_INCREMENT = 0.01;
  static constexpr float UPSCALE_LIMIT = 0.75;
  static constexpr double MAX_FIT = 1e5;
  static constexpr double DILATION_FACTOR = 9;
  static constexpr double CLOUD_RESOLUTION = 20.0;
  static constexpr int MIN_MATCH_SIZE = 3500;
  static constexpr double MAX_DISTANCE = 3;
  static constexpr int COUNTER_THRESHOLD = 2;
  static constexpr double LOOP_RATE = 0.1;
  static constexpr float SCALING_FACTOR = 1000.0;
  static constexpr auto WARN_TIME = 5;
};

using namespace ros_util;
using PCXYZ = pcl::PointCloud<pcl::PointXYZ>;
using ROSCloud = sensor_msgs::PointCloud2;
using DetectionConfig = pointcloud_filter::WallDetectionParametersConfig;

class WallDetection {
public:
  explicit WallDetection(ros::NodeHandle &t_nh)
      : m_handlerMapCloud(t_nh, "submap_cloud"), m_g2l(t_nh) {
    initialize_parameters(t_nh);
    m_pubFilteredCloud = t_nh.advertise<ROSCloud>("filtered_cloud", 1);
    m_pubTargetCloud = t_nh.advertise<ROSCloud>("target_cloud", 1);
    m_pubAlignedCloud = t_nh.advertise<ROSCloud>("aligned_cloud", 1);
    m_pubWallOdometry = t_nh.advertise<nav_msgs::Odometry>("wall/odometry", 1);

    image_transport::ImageTransport it(t_nh);
    m_pubPointCloudImage = it.advertise("cloud_image", 1);
    m_pubResultImage = it.advertise("result_image", 1);
    m_pubWallTargetImage = it.advertise("target_image", 1);

    m_loopTimer = t_nh.createTimer(WallDetectionParameters::LOOP_RATE,
                                   &WallDetection::loop_event, this);

    m_targetWallCloud = boost::make_shared<PCXYZ>();
    wall_from_ply(m_targetWallCloud,
                  "config/zid_tanko_upscale.ply"); //"config/zid_single.ply");
    m_targetWallCloud = organize_pointcloud(m_targetWallCloud);
    m_targetWallImage = PCXYZ_to_cvMat(m_targetWallCloud);
  }

private:
  void do_icp(const PCXYZ::Ptr &t_inputCloud, const PCXYZ::Ptr &t_targetCloud) {
    if (t_inputCloud->empty()) {
      ROS_WARN_THROTTLE(WallDetectionParameters::WARN_TIME,
                        "WallDetection::do_icp - empty cloud");
      return;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(t_targetCloud);
    icp.setInputTarget(t_inputCloud);
    auto alignedCloud = boost::make_shared<PCXYZ>();
    icp.align(*alignedCloud);
    ROS_INFO_COND(icp.hasConverged(),
                  "WallDetection::do_icp - ICP converged. Score: [%.2f]",
                  icp.getFitnessScore());
    ROS_FATAL_COND(!icp.hasConverged(),
                   "WallDetection::do_icp - ICP did not converge. :(");

    if (!icp.hasConverged()) {
      return;
    }

    // Do some stuff if converged
    ROS_INFO_STREAM("ICP transformation: " << icp.getFinalTransformation());
    publish_cloud(alignedCloud, m_pubAlignedCloud);
  }

  void wall_from_ply(PCXYZ::Ptr &t_wallCloud, const std::string &plyPath) {
    auto wallCloud = boost::make_shared<PCXYZ>();
    std::string path =
        ros::package::getPath("pointcloud_filter") + "/" + plyPath;
    ROS_INFO("WallDetection - %s", path.c_str());
    if (pcl::io::loadPLYFile(path, *wallCloud) == -1) {
      ROS_FATAL("WallDetection - unable to load whe wall mesh, exiting...");
      throw std::runtime_error("WallDetection - unable to load wall mesh");
    }

    auto wallWithMean = boost::make_shared<PCXYZ>();
    for (const auto &point : wallCloud->points) {
      wallWithMean->points.emplace_back(
          pcl::PointXYZ(point.x / WallDetectionParameters::SCALING_FACTOR,
                        point.y / WallDetectionParameters::SCALING_FACTOR,
                        point.z / WallDetectionParameters::SCALING_FACTOR));
    }

    // Upscale the wall
    static constexpr double INITIAL_POSITION = 0.01;
    static constexpr auto ITER_COUNT =
        static_cast<int>(WallDetectionParameters::UPSCALE_LIMIT /
                         WallDetectionParameters::UPSCALE_INCREMENT);
    for (int i = 0; i < ITER_COUNT; i++) {
      for (int j = 0; j < ITER_COUNT; j++) {
        for (const auto &point : wallCloud->points) {
          wallWithMean->points.emplace_back(
              pcl::PointXYZ(point.x / WallDetectionParameters::SCALING_FACTOR +
                                INITIAL_POSITION +
                                i * WallDetectionParameters::UPSCALE_INCREMENT,
                            point.y / WallDetectionParameters::SCALING_FACTOR +
                                INITIAL_POSITION +
                                j * WallDetectionParameters::UPSCALE_INCREMENT,
                            point.z / WallDetectionParameters::SCALING_FACTOR));
        }
      }
    }

    // Final wall cloud is the demeaned cloud
    t_wallCloud = demean_cloud(wallWithMean);
    m_lastFoundMapCentroid = Eigen::Vector4f{0, 0, 0, 0};
  }

  void add_wall_position(double x, double y) {
    auto newPosition = std::make_pair(x, y);
    auto success = m_wallPositionMap.emplace(newPosition, 0);
    if (!success.second) {
      m_wallPositionMap[newPosition]++;
    }

    ROS_INFO("PickupChallengeInfo - wall at [%.2f, %.2f] seen %lu times.", x, y,
             m_wallPositionMap[newPosition]);
  }

  bool get_wall_position(std::pair<double, double> &t_wallPosition) {
    double maxValue = 0;
    double meanValue = 0;
    std::pair<double, double> bestWallPosition;
    for (const auto &wallPosition : m_wallPositionMap) {
      meanValue += wallPosition.second;

      if (wallPosition.second > maxValue) {
        bestWallPosition = wallPosition.first;
        maxValue = wallPosition.second;
      }
    }

    meanValue /= m_wallPositionMap.size();
    static constexpr auto TOL = 5;
    if (maxValue - meanValue < TOL) {
      return false;
    }

    t_wallPosition = bestWallPosition;
    return true;
  }

  static cv::Mat PCXYZ_to_cvMat(const PCXYZ::Ptr &t_inputCloud) {
    if (t_inputCloud->empty()) {
      ROS_INFO("PCXYZ_to_cvMAT - input cloud is empty");
      return cv::Mat();
    }

    if (!t_inputCloud->isOrganized()) {
      ROS_FATAL("PXCYZ_to_cvMAT - cannot convert an unorganized pointcloud");
      return cv::Mat();
    }

    ROS_INFO_COND(t_inputCloud->isOrganized(),
                  "WallDetection::PCXYZ_to_cvMat - cloud is organized");
    ROS_INFO("Size: [%d, %d]", t_inputCloud->height, t_inputCloud->width);

    static constexpr auto WHITE_VAL = 255;
    cv::Mat outImage(t_inputCloud->height, t_inputCloud->width, CV_8UC1,
                     cv::Scalar(0));
    for (int x = 0; x < outImage.rows; x++) {
      for (int y = 0; y < outImage.cols; y++) {
        auto point = t_inputCloud->at(x, y);
        if (point.x == 0 && point.y == 0 && point.z == 0) {
          continue;
        }
        outImage.at<uchar>(x, y) = WHITE_VAL;
      }
    }
    return outImage;
  }

  Eigen::Matrix4f template_matching(const cv::Mat &t_source8UC1,
                                    const cv::Mat &t_target8UC1) {
    if (t_source8UC1.empty()) {
      ROS_WARN_THROTTLE(WallDetectionParameters::WARN_TIME,
                        "template_matching - source is empty.");
      return {};
    }

    cv::Mat resultImage(t_source8UC1);
    double dilation_size = m_handlerParam->getData().dilation_factor;
    cv::Mat element = getStructuringElement(
        cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
        cv::Point(dilation_size, dilation_size));
    cv::dilate(resultImage, resultImage, element);

    using ContourVector = std::vector<std::vector<cv::Point>>;
    ContourVector sourceContours;
    ContourVector targetContours;
    cv::findContours(resultImage, sourceContours, CV_RETR_LIST,
                     CV_CHAIN_APPROX_SIMPLE);
    cv::findContours(t_target8UC1, targetContours, CV_RETR_LIST,
                     CV_CHAIN_APPROX_SIMPLE);

    ROS_INFO("Source contour count: %lu", sourceContours.size());
    ROS_INFO("Target contour count: %lu", targetContours.size());

    int bestMatchedIndex = -1;
    static constexpr auto INITIAL_FITNESS = 1e5;
    double bestFitness = INITIAL_FITNESS;
    for (int i = 0; i < sourceContours.size(); i++) {

      if (cv::contourArea(sourceContours[i]) <
          m_handlerParam->getData().min_match_size) {
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
      ROS_FATAL("WallDetection::template_matching - unable to match contours");
      return {};
    }

    ROS_INFO("Best fitness: %.5f", bestFitness);
    static constexpr auto DRAW_COLOR = 150;
    static constexpr auto DRAW_THICKNESS = 5;
    cv::drawContours(resultImage, sourceContours, bestMatchedIndex,
                     cv::Scalar(DRAW_COLOR), DRAW_THICKNESS);
    cv::Moments M = cv::moments(sourceContours[bestMatchedIndex]);
    int cX = int(M.m10 / M.m00);
    int cY = int(M.m01 / M.m00);
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(1, 3) = (cX - t_source8UC1.rows / 2.0) /
                          WallDetectionParameters::CLOUD_RESOLUTION +
                      m_lastFoundMapCentroid.y();
    transform(0, 3) = (cY - t_source8UC1.cols / 2.0) /
                          WallDetectionParameters::CLOUD_RESOLUTION +
                      m_lastFoundMapCentroid.x();
    transform(2, 3) = m_lastFoundMapCentroid.z();
    ROS_INFO_STREAM("Target transform: " << transform);

    double distance_from_previous =
        sqrt(pow(transform(0, 3) - m_bestTransformation(0, 3), 2) +
             pow(transform(1, 3) - m_bestTransformation(1, 3), 2));

    if (distance_from_previous > WallDetectionParameters::MAX_DISTANCE) {
      ROS_WARN("WallDetection - resetting detection counter.");
      m_detectionCounter = 0;
    } else {
      m_detectionCounter++;
      ROS_INFO("WallDetection - counter at %d", m_detectionCounter);
    }
    m_bestTransformation = transform;

    publish_image(resultImage, m_pubResultImage);
    publish_image(t_target8UC1, m_pubWallTargetImage);
    publish_wall_odometry(m_bestTransformation);

    // add_wall_position(m_bestTransformation(0, 3), m_bestTransformation(1,
    // 3)); std::pair<double, double> wallPosition;
    if (m_detectionCounter >= WallDetectionParameters::COUNTER_THRESHOLD) {
      auto wallGlobal = m_g2l.toGlobal(m_bestTransformation(0, 3),
                                       m_bestTransformation(1, 3), 0);
      m_nh.setParam("brick_dropoff/lat", wallGlobal.x());
      m_nh.setParam("brick_dropoff/lon", wallGlobal.y());
      ROS_WARN("Set new pickup position! [%.10f, %.10f]", wallGlobal.x(),
               wallGlobal.y());
    }
    return transform;
  }

  static PCXYZ::Ptr organize_pointcloud(const PCXYZ::Ptr &t_unorganizedCloud,
                                        const int t_width = 100,
                                        const int t_height = 100) {
    if (t_unorganizedCloud->empty()) {
      return t_unorganizedCloud;
    }

    auto organizedCloud = boost::make_shared<PCXYZ>(
        t_width * WallDetectionParameters::CLOUD_RESOLUTION,
        t_height * WallDetectionParameters::CLOUD_RESOLUTION,
        pcl::PointXYZ(0, 0, 0));

    const auto out_of_range = [&organizedCloud](const std::size_t t_x,
                                                const std::size_t t_y) {
      return t_x >= organizedCloud->width || t_y >= organizedCloud->height;
    };
    const auto is_point_empty = [](const pcl::PointXYZ &point) {
      return point.x == 0 && point.y == 0 && point.z == 0;
    };
    const std::size_t indexOffsetX = round(organizedCloud->width / 2.0);
    const std::size_t indexOffsetY = round(organizedCloud->height / 2.0);
    const auto add_to_organized_cloud = [&](const pcl::PointXYZ &point) {
      const std::size_t indX =
          round(point.x * WallDetectionParameters::CLOUD_RESOLUTION) +
          indexOffsetX;
      const std::size_t indY =
          round(point.y * WallDetectionParameters::CLOUD_RESOLUTION) +
          indexOffsetY;

      if (out_of_range(indX, indY)) {
        return;
      }

      auto currentPoint = organizedCloud->at(indX, indY);
      if (is_point_empty(currentPoint) || point.z > currentPoint.z) {
        organizedCloud->at(indX, indY) = point;
      }
    };

    std::for_each(t_unorganizedCloud->points.begin(),
                  t_unorganizedCloud->points.end(), add_to_organized_cloud);

    return organizedCloud;
  }

  void publish_wall_odometry(const Eigen::Matrix4f &t_transform) {
    const auto get_translation_x = [&t_transform]() {
      return t_transform(0, 3);
    };
    const auto get_translation_y = [&t_transform]() {
      return t_transform(1, 3);
    };
    const auto get_translation_z = [&t_transform]() {
      return t_transform(2, 3);
    };
    const auto get_transform_quaternion = [&t_transform]() {
      tf::Matrix3x3 mat(t_transform(0, 0), t_transform(0, 1), t_transform(0, 2),
                        t_transform(1, 0), t_transform(1, 1), t_transform(1, 2),
                        t_transform(2, 0), t_transform(2, 1),
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

  PCXYZ::Ptr do_outlier_filtering(const PCXYZ::Ptr &t_inputCloud) {
    auto filteredCloud = boost::make_shared<PCXYZ>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(t_inputCloud);
    sor.setMeanK(m_handlerParam->getData().outlier_filter_mean);
    sor.setStddevMulThresh(m_handlerParam->getData().outlier_filter_stddev);
    sor.filter(*filteredCloud);
    return filteredCloud;
  }

  static PCXYZ::Ptr decrease_by_min_height(const PCXYZ::Ptr &t_input) {
    if (t_input->empty()) {
      return t_input;
    }

    pcl::PointXYZ minPoint;
    pcl::PointXYZ maxPoint;
    pcl::getMinMax3D(*t_input, minPoint, maxPoint);
    auto outCloud = boost::make_shared<PCXYZ>();
    for (const auto &point : t_input->points) {
      outCloud->points.emplace_back(
          pcl::PointXYZ(point.x, point.y, point.z - minPoint.z));
    }
    return outCloud;
  }

  PCXYZ::Ptr demean_cloud(const PCXYZ::Ptr &t_cloudWithMean) {
    if (t_cloudWithMean->empty()) {
      return t_cloudWithMean;
    }

    auto outCloud = boost::make_shared<PCXYZ>();
    pcl::compute3DCentroid(*t_cloudWithMean, m_lastFoundMapCentroid);
    for (const auto &point : t_cloudWithMean->points) {
      outCloud->points.emplace_back(
          pcl::PointXYZ(point.x - m_lastFoundMapCentroid.x(),
                        point.y - m_lastFoundMapCentroid.y(),
                        point.z - m_lastFoundMapCentroid.z()));
    }
    return outCloud;
  }

  void initialize_parameters(ros::NodeHandle & /*unused*/) {
    DetectionConfig config;
    config.outlier_filter_mean = WallDetectionParameters::OUTLIER_MEAN;
    config.outlier_filter_stddev = WallDetectionParameters::OUTLIER_STDDEV;
    config.min_crop_height = WallDetectionParameters::MIN_HEIGHT;
    config.max_crop_height = WallDetectionParameters::MAX_HEIGHT;
    config.dilation_factor = WallDetectionParameters::DILATION_FACTOR;
    config.min_match_size = WallDetectionParameters::MIN_MATCH_SIZE;
    m_handlerParam = std::make_shared<ParamHandler<DetectionConfig>>(
        config, "wall_detection");
  }

  void loop_event(const ros::TimerEvent & /*unused*/) {
    // Convert from ROS message to pcl
    auto inputCloud = boost::make_shared<PCXYZ>();
    read_input_cloud(inputCloud);
    inputCloud = crop_by_height(inputCloud);
    inputCloud = do_outlier_filtering(inputCloud);
    inputCloud = demean_cloud(inputCloud);
    inputCloud = organize_pointcloud(inputCloud);
    auto image = PCXYZ_to_cvMat(inputCloud);
    auto targetTransform = template_matching(image, m_targetWallImage);

    // auto transformedTargetCloud = boost::make_shared<PCXYZ>();
    // pcl::transformPointCloud(*m_targetWallCloud, *transformedTargetCloud,
    // targetTransform); do_icp(inputCloud, transformedTargetCloud); // TODO:
    // Try dilating for ICP aswell

    publish_image(image, m_pubPointCloudImage);
    publish_cloud(inputCloud, m_pubFilteredCloud);
    publish_cloud(m_targetWallCloud, m_pubTargetCloud);
  }

  PCXYZ::Ptr crop_by_height(const PCXYZ::ConstPtr &t_inputCloud) {
    if (t_inputCloud->empty()) {
      return boost::make_shared<PCXYZ>();
    }

    auto newCloud = boost::make_shared<PCXYZ>();
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(
        -WallDetectionParameters::MAX_BOX, -WallDetectionParameters::MAX_BOX,
        m_handlerParam->getData().min_crop_height, 0.0));
    boxFilter.setMax(Eigen::Vector4f(
        WallDetectionParameters::MAX_BOX, WallDetectionParameters::MAX_BOX,
        m_handlerParam->getData().max_crop_height, 0.0));
    boxFilter.setInputCloud(t_inputCloud);
    boxFilter.filter(*newCloud);
    return newCloud;
  }

  void publish_image(const cv::Mat &t_image,
                     image_transport::Publisher &t_pub) {
    cv_bridge::CvImage bridgeImage;
    bridgeImage.header.frame_id = ros::this_node::getNamespace() + "/map";
    bridgeImage.header.stamp = ros::Time::now();
    bridgeImage.header.seq = m_handlerMapCloud.getData().header.seq;
    bridgeImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    bridgeImage.image = t_image;
    t_pub.publish(bridgeImage.toImageMsg());
  }

  inline void read_input_cloud(PCXYZ::Ptr &t_inputCloud) {
    pcl::fromROSMsg(m_handlerMapCloud.getData(), *t_inputCloud);
  }

  static void publish_cloud(const PCXYZ::Ptr &t_inputCloud,
                            ros::Publisher &t_pub) {
    auto rosMsg = boost::make_shared<ROSCloud>();
    pcl::toROSMsg(*t_inputCloud, *rosMsg);
    rosMsg->header.frame_id = ros::this_node::getNamespace() + "/map";
    rosMsg->header.stamp = ros::Time::now();
    t_pub.publish(*rosMsg);
  }

  ros::Timer m_loopTimer;
  Eigen::Vector4f m_lastFoundMapCentroid;
  Eigen::Matrix4f m_bestTransformation;
  double m_maxFitness = WallDetectionParameters::MAX_FIT;

  ros::Publisher m_pubFilteredCloud, m_pubTargetCloud, m_pubAlignedCloud,
      m_pubWallOdometry;
  image_transport::Publisher m_pubPointCloudImage, m_pubResultImage,
      m_pubWallTargetImage;

  TopicHandler<ROSCloud> m_handlerMapCloud;
  std::shared_ptr<ParamHandler<DetectionConfig>> m_handlerParam;
  PCXYZ::Ptr m_targetWallCloud;
  cv::Mat m_targetWallImage;
  std::unordered_map<std::pair<double, double>, std::size_t, PairHash>
      m_wallPositionMap;

  Global2Local m_g2l;
  ros::NodeHandle m_nh;
  int m_detectionCounter = 0;
};

} // namespace detection
#endif /* WALL_DETECTION_H */