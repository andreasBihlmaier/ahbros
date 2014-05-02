#ifndef _AHB_ROS_H_
#define _AHB_ROS_H_

#include <string>
#include <sstream>
#include <iomanip>
#include <vector>

#include <tf/transform_datatypes.h>

#include <ahbstring.h>


namespace ahb {
namespace string {

/**
 * Convert tf::Vector3 to string
 */
template<> std::string toString<tf::Vector3>(const tf::Vector3& p_vec)
{
  std::stringstream ss;

  ss << "(" << p_vec.x() << ", " << p_vec.y() << ", " << p_vec.z() << ")";

  return ss.str();
}

/**
 * Convert tf::Quaternion to string
 */
template<> std::string toString<tf::Quaternion>(const tf::Quaternion& p_quat)
{
  std::stringstream ss;

  ss << "(" << p_quat.x() << ", " << p_quat.y() << ", " << p_quat.z() << ", " << p_quat.w() << ")";

  return ss.str();
}

/**
 * Convert tf::Pose to string
 */
template<> std::string toString<tf::Pose>(const tf::Pose& p_pose)
{
  std::stringstream ss;

  ss << "Pose(Quaternion=" << toString(p_pose.getRotation()) << "; Vector3=" << toString(p_pose.getOrigin()) << ")";

  return ss.str();
}

} // namespace: string

namespace ros {

} // namespace: ros
} // namespace: ahb

#endif // _AHB_ROS_H_
