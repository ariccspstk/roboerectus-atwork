#ifndef ARICC_2D_VISION_RTTIMAGE_H_
#define ARICC_2D_VISION_RTTIMAGE_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <image_geometry/pinhole_camera_model.h>
#include <dynamic_reconfigure/server.h>
#include <aricc_2d_vision/RTTImageConfig.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace aricc_2d_vision
{
  class RTTImage: public aricc_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef aricc_2d_vision::RTTImageConfig Config;

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
  
  private:
    void configCallback(Config &config, uint32_t level);
    void infoCallback(
      const sensor_msgs::Image::ConstPtr& msg);
   
    bool processImage(cv::Mat image);
    void publishResult();

    boost::mutex mutex_;
    ros::Publisher pub_;
    ros::Subscriber sub_info_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;

    double threshold_;
  };
}

#endif
