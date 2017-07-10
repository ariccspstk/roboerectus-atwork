#ifndef ARICC_2D_VISION_CREATE_THRESHOLD_IMAGE_H_
#define ARICC_2D_VISION_CREATE_THRESHOLD_IMAGE_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <aricc_2d_vision/CreateThresholdImageConfig.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>

namespace aricc_2d_vision
{
  class CreateThresholdImage: public aricc_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef aricc_2d_vision::CreateThresholdImageConfig Config;

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
    unsigned int block_size_;
    unsigned int constant_;
    unsigned int method_;
    unsigned int type_;
    unsigned int value_;
    
  private:
    
  };
}

#endif
