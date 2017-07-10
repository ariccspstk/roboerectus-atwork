#include <aricc_utils/geometry_utils.h>

namespace aricc_utils{

  void quat2Euler(geometry_msgs::Quaternion quat_msg, double& roll, double& pitch, double& yaw){
    tf::Quaternion q;
    tf::quaternionMsgToTF(quat_msg, q );
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
  }
  void euler2Quat(double roll, double pitch, double yaw,
    geometry_msgs::Quaternion& quat_msg){
    tf::Quaternion q;
    q = tf::createQuaternionFromRPY(roll,pitch,yaw);
    tf::quaternionTFToMsg(q,quat_msg);
  }

  // Normalize an angle to be between -pi and pi
  double normalizeAngle(double r) {
    while (r <= -M_PI) r += 2*M_PI;
    while (r > M_PI) r -= 2*M_PI;
    return r;
  }
  
  // Normalize an angle to be between -pi/2 and pi/2
  double smallAngle(double r) {
    r = normalizeAngle(r);
    if (r > M_PI_2) r -= M_PI;
    if (r < -M_PI_2) r += M_PI;
    return r;
  }

  double sign(double value){
    if (value >= 0) return 1.0;
    else return -1.0;
  }
  
  double deg2Rad(double degree){
    double r = degree/180.0*M_PI;
    return r;
  }

  double rad2Deg(double radius){
    double d = radius/M_PI*180.0;
    return d;
  }

}
