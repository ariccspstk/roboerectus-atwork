#ifndef ARICC_2D_VISION_SMOOTHING_IMAGE_H_
#define ARICC_2D_VISION_SMOOTHING_IMAGE_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <aricc_2d_vision/SmoothingImageConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>

namespace aricc_2d_vision
{
  class SmoothingImage: public aricc_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef aricc_2d_vision::SmoothingImageConfig Config;

  protected:

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config &config, uint32_t level);
    virtual void execute(const sensor_msgs::Image::ConstPtr& image_msg);

    ros::Subscriber sub_;
    ros::Publisher pub_;
    boost::mutex mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    unsigned int method_;
    unsigned int kernel_size_;
  private:
    
  };
}

#endif
