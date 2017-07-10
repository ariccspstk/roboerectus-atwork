#ifndef ARICC_2D_VISION_ROTATED_RECT_FINDER_H_
#define ARICC_2D_VISION_ROTATED_RECT_FINDER_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <aricc_vision_msgs/RotatedRectArray.h>
#include <aricc_vision_msgs/ContourArray.h>
#include <aricc_2d_vision/RotatedRectFinderConfig.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace aricc_2d_vision{

  class RotatedRectFinder: public aricc_topic_tools::ConnectionBasedNodelet{
  public:
    typedef aricc_2d_vision::RotatedRectFinderConfig Config;
    typedef message_filters::sync_policies::ExactTime<
    aricc_vision_msgs::ContourArray,
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo > SyncPolicy;

    typedef message_filters::sync_policies::ApproximateTime<
    aricc_vision_msgs::ContourArray,
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo > ApproxSyncPolicy;
    
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
   
  private:
    boost::mutex mutex_;
    message_filters::Subscriber<aricc_vision_msgs::ContourArray> sub_contour_;
    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_image_;
    message_filters::Subscriber<sensor_msgs::Image> sub_threshold_image_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_image_info_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy> > async_;
    ros::Publisher pub_rects_;
    ros::Publisher pub_rects_p_;
    ros::Publisher pub_debug_image_;
    image_geometry::PinholeCameraModel model_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    
    bool debug_;
    bool result_;
    bool ruler_;
    bool detect_color_;
    bool detect_density_;
    bool project_;
    bool approximate_sync_;
    int img_width_;
    int img_height_;
    double z_;
    aricc_vision_msgs::RotatedRectArray msg_rects_;
    aricc_vision_msgs::RotatedRectArray msg_rects_p_;
 
    void execute(
      const aricc_vision_msgs::ContourArray::ConstPtr& msg,
      const sensor_msgs::Image::ConstPtr& rgb_image_msg,
      const sensor_msgs::Image::ConstPtr& threshold_image_msg,
      const sensor_msgs::CameraInfo::ConstPtr& image_info_msg);

    void configCallback(Config &config, uint32_t level);   
    double findDensity(std::vector<cv::Point> contour, 
                     cv::Mat image);
    void pubDebug(cv::Mat& src, std_msgs::Header header);
    void getDim(aricc_vision_msgs::RotatedRect& rect);
    double getDist(geometry_msgs::Point p1, geometry_msgs::Point p2);
    void drawRuler(cv::Mat& src);
    void drawResult(cv::Mat& src);
    void drawRect( std::vector<cv::Point> contour, 
      cv::Mat& image);
    void toROSMsg( 
      cv::RotatedRect rect,
      double color_h,
      double color_s,
      double color_v,
      double density,
      aricc_vision_msgs::RotatedRect& msg);
    void project(
      aricc_vision_msgs::RotatedRect rect,
      aricc_vision_msgs::RotatedRect& rect_p );
  };
}

#endif
