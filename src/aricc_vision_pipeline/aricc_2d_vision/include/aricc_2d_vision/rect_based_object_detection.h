#ifndef ARICC_2D_VISION_RECT_BASED_OBJECT_DETECTION_H_
#define ARICC_2D_VISION_RECT_BASED_OBJECT_DETECTION_H_

#include <aricc_vision_msgs/RotatedRectArray.h>
#include <aricc_vision_msgs/RotatedRect.h>
#include <aricc_vision_msgs/Object.h>
#include <aricc_vision_msgs/ObjectArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <aricc_2d_vision/RectBasedObjectDetectionConfig.h>
#include <aricc_topic_tools/connection_based_nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>



struct Object{
  std::string name;
  double width;
  double height;
  double color_h;
  double color_s;
  double color_v;
  double density;
};

namespace aricc_2d_vision{

  class RectBasedObjectDetection: public aricc_topic_tools::ConnectionBasedNodelet{
  public:
    typedef RectBasedObjectDetectionConfig Config;

    typedef message_filters::sync_policies::ExactTime<
    aricc_vision_msgs::RotatedRectArray,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo > SyncPolicy;

    typedef message_filters::sync_policies::ApproximateTime<
    aricc_vision_msgs::RotatedRectArray,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo > ApproxSyncPolicy;
 
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    ros::Publisher pub_;
    ros::Publisher pub_debug_image_;
    
  private:
    boost::mutex mutex_;
    message_filters::Subscriber<aricc_vision_msgs::RotatedRectArray> sub_rects_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_image_info_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy> > async_;

    std::vector<Object> object_list_;
    double width_tolerance_;
    double height_tolerance_;
    double color_h_tol_;
    double color_s_tol_;
    double color_v_tol_;
    double density_tolerance_;
    std::string list_title_;
    bool debug_;
    bool print_info_;
    bool approximate_sync_;
    image_geometry::PinholeCameraModel model_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    
    void execute(
      const aricc_vision_msgs::RotatedRectArray::ConstPtr& msg,
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::CameraInfo::ConstPtr& image_info_msg);
    void configCallback(Config& config, uint32_t level);
    void loadObjects();
    std::string detect(aricc_vision_msgs::RotatedRect rect);
    void pubDebug(cv::Mat& src, 
      aricc_vision_msgs::ObjectArray msg_objects,
      aricc_vision_msgs::RotatedRectArray msg_rects);
  };
}

#endif
