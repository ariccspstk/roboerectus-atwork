#ifndef ARICC_2D_VISION_RECT_TO_MASK_IMAGE_H_
#define ARICC_2D_VISION_RECT_TO_MASK_IMAGE_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <dynamic_reconfigure/server.h>
#include <aricc_2d_vision/RectToMaskImageConfig.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace aricc_2d_vision
{
  class RectToMaskImage: public aricc_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef aricc_2d_vision::RectToMaskImageConfig Config;

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
  
  private:
    void configCallback(Config &config, uint32_t level);
    void infoCallback(
      const sensor_msgs::CameraInfo::ConstPtr& info_msg);

    void createProjectMask(cv::Mat& mask_image,
    sensor_msgs::CameraInfo info_msg);

    void createMask(cv::Mat& mask_image,
    sensor_msgs::CameraInfo info_msg);

    boost::mutex mutex_;
    ros::Publisher pub_;
    ros::Subscriber sub_info_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;

    double left_top_x_;
    double left_top_y_;
    double width_;
    double height_;
    double z_;
    bool project_;
  };
}

#endif
