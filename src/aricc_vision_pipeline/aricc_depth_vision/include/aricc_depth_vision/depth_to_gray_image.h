#ifndef ARICC_2D_VISION_CREATE_THRESHOLD_IMAGE_H_
#define ARICC_2D_VISION_CREATE_THRESHOLD_IMAGE_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <aricc_depth_vision/DepthToGrayImageConfig.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>

namespace aricc_depth_vision
{
  class DepthToGrayImage: public aricc_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef aricc_depth_vision::DepthToGrayImageConfig Config;

  protected:

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config &config, uint32_t level);
    virtual void execute(const sensor_msgs::Image::ConstPtr& image_msg);

    boost::mutex mutex_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    float min_range_;
    float max_range_;
    
  private:
    
  };
}

#endif
