#ifndef UTM_CONVERTER
#define UTM_CONVERTER

#include <GeographicLib/UTMUPS.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose2D.h>
#include <GPS_Out.h>

namespace usrg_utm {

class UtmProjector 
{
 public:
  UtmProjector(sensor_msgs::NavSatFix& origin);
  geometry_msgs::Pose2D forward(const sensor_msgs::NavSatFix& gps);
  sensor_msgs::NavSatFix reverse(const geometry_msgs::Pose2D& utm);

  sensor_msgs::NavSatFix m_origin;
  geometry_msgs::Pose2D m_utmOrigin;

 private:
  int m_zone;
  bool m_isInNorthernHemisphere;
};

} //usrg_utm 

#endif