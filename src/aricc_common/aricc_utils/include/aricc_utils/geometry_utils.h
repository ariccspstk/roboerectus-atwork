#ifndef ARICC_UTILS_ARICC_UTILS_
#define ARICC_UTILS_ARICC_UTILS_
#include <tf/tf.h>
#include <angles/angles.h>
#include <math.h>
#include <geometry_msgs/Quaternion.h>

namespace aricc_utils{

  void quat2Euler(geometry_msgs::Quaternion quat_msg, 
    double& roll, double& pitch, double& yaw);
  void euler2Quat(double roll, double pitch, double yaw,
    geometry_msgs::Quaternion& quat_msg);
  double normalizeAngle(double r);
  double smallAngle(double r);
  double sign(double value);
  double deg2Rad(double degree);
  double rad2Deg(double radius);


}

#endif

