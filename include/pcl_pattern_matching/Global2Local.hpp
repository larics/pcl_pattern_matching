#ifndef GLOBAL_TO_LOCAL_HPP
#define GLOBAL_TO_LOCAL_HPP

#include <ros/ros.h>
#include "pcl_pattern_matching/Util.hpp"
#include <Eigen/Dense>
#include <GeographicLib/Geocentric.hpp>
#include <mavros/frame_tf.h>
#include <mavros_msgs/HomePosition.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace ros_util;

class Global2Local {
public:
Global2Local(ros::NodeHandle& nh) :
    m_homeHandler(nh, "mavros/global_position/home", -1) { }

Eigen::Vector3d toGlobal(const double t_enuX, const double t_enuY, const double t_enuZ)
{
  Eigen::Vector3d localEnu{t_enuX, t_enuY, t_enuZ};
  Eigen::Vector3d map_origin(
      m_homeHandler.getData().geo.latitude, 
      m_homeHandler.getData().geo.longitude, 
      m_homeHandler.getData().geo.altitude);
  Eigen::Vector3d localEcef = mavros::ftf::transform_frame_enu_ecef(localEnu, map_origin);
  
  GeographicLib::Geocentric earth(
    GeographicLib::Constants::WGS84_a(),
    GeographicLib::Constants::WGS84_f()
  );

  Eigen::Vector3d ecef_origin;
  earth.Forward(
    map_origin.x(), map_origin.y(), map_origin.z(),
    ecef_origin.x(), ecef_origin.y(), ecef_origin.z());
    
  localEcef = localEcef + ecef_origin;

  Eigen::Vector3d geoPosition;
  earth.Reverse(localEcef.x(), localEcef.y(), localEcef.z(),
    geoPosition.x(), geoPosition.y(), geoPosition.z());
  return geoPosition;
}

Eigen::Vector3d toLocal(const double lat, const double lon, const double alt, bool altitudeRelative = false) {
  Eigen::Vector3d local_ecef;
  try {
    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
            GeographicLib::Constants::WGS84_f());

    Eigen::Vector3d ecef_origin, 
    map_origin(
      m_homeHandler.getData().geo.latitude, 
      m_homeHandler.getData().geo.longitude, 
      m_homeHandler.getData().geo.altitude);
    
    earth.Forward(
      map_origin.x(), map_origin.y(), map_origin.z(),
      ecef_origin.x(), ecef_origin.y(), ecef_origin.z());
    
    earth.Forward(lat, lon, map_origin.z(),
      local_ecef.x(), local_ecef.y(), local_ecef.z());   
    
    local_ecef = local_ecef - ecef_origin;
    local_ecef = mavros::ftf::transform_frame_ecef_enu(local_ecef, map_origin);
  } catch (const std::exception& e) {
    ROS_FATAL("Global2Local::toLocal - unable to perform transformation");
  }

  if (altitudeRelative) {
    local_ecef.z() = alt;
  }

  if (!m_homeHandler.isMessageRecieved()) {
    ROS_FATAL("Global2Local - unable to get home position.");
    local_ecef.x() = 0;
    local_ecef.y() = 0;
  }

  return local_ecef;
}

private:
TopicHandler<mavros_msgs::HomePosition> m_homeHandler;
};
#endif