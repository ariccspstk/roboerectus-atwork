#include "aricc_3d_vision/border_estimator.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "border_estimator_test");
  aricc_3d_vision::BorderEstimator borderEstimator;
  borderEstimator.spin();
  
  ros::shutdown();
  return 0;
}
