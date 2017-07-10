#ifndef ARICC_2D_VISION_CHECK_BOARD_DETECTOR_H_
#define ARICC_2D_VISION_CHECK_BOARD_DETECTOR_H_

#include <algorithm>
#include <cstdio>
#include <vector>
#include <sstream>
#include <boost/thread/mutex.hpp>
#include "math.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <aricc_2d_vision/CheckBoardDetectorConfig.h>
#include <aricc_utils/geometry_utils.h>

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

struct CheckBoard{
  std::vector<cv::Point2f> corners;
  tf::Transform tf;
};

namespace aricc_2d_vision{
  class CheckBoardDetector: public aricc_topic_tools::ConnectionBasedNodelet{
  public:
    typedef aricc_2d_vision::CheckBoardDetectorConfig Config;

    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image,
    sensor_msgs::CameraInfo > SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::CameraInfo > ApproxSyncPolicy;
    
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
   
  private:
    void execute(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::CameraInfo::ConstPtr& info_msg);

    bool detect(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::CameraInfo::ConstPtr& info_msg);

    tf::Transform findTransformation(
      const std::vector<cv::Point2f>& img_points,
      const std::vector<cv::Point3f>& grid_points,
      const image_geometry::PinholeCameraModel model);

    void displayResult(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::CameraInfo::ConstPtr& info_msg);

    void publishResult(std_msgs::Header);
    void configCallback(Config &config, uint32_t level); 

    bool approximate_sync_;
    bool debug_;
    bool verbose_;
    bool invert_color_;
    bool use_P_;
    int max_board_;
    int columns_;
    int rows_;
    int  border_thresh_;
    double rect_size_x_;
    double rect_size_y_;
    std::string board_type_;
    std::string camera_link_;
    std::vector<cv::Point3f> grid3d_;
    TransformMatrix tf_ROS_;
    //std::vector<CheckBoard> detected_boards_;
    CheckBoard detected_boards_;
    boost::mutex mutex_;    
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;

    ros::Publisher pub_debug_;
    ros::Publisher pub_;

    message_filters::Subscriber<sensor_msgs::Image> sub_camera_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy> > async_;
    
  };
}

#endif
