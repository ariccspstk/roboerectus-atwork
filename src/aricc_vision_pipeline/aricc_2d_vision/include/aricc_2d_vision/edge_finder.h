#ifndef ARICC_2D_VISION_EDGE_FINDER_H_
#define ARICC_2D_VISION_EDGE_FINDER_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <aricc_2d_vision/EdgeFinderConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>

namespace aricc_2d_vision
{
  class EdgeFinder: public aricc_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef aricc_2d_vision::EdgeFinderConfig Config;

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
    double low_threshold_;
    double ratio_;
    int    kernel_size_;
    bool   L2_gradient_;
  private:
    
  };
}

#endif
