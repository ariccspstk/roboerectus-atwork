#ifndef ARICC_2D_VISION_COLOUR_FINDER_H_
#define ARICC_2D_VISION_COLOUR_FINDER_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <aricc_2d_vision/ColourFinderConfig.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace aricc_2d_vision{

  class ColourFinder: public aricc_topic_tools::ConnectionBasedNodelet{
  public:
    typedef aricc_2d_vision::ColourFinderConfig Config;

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
   
  private:
    boost::mutex mutex_;
    ros::Subscriber sub_;

    ros::Publisher pub_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    
    int h_high_;
    int h_low_;
    int s_high_;
    int s_low_;
    int v_high_;
    int v_low_;
 
    void execute(const sensor_msgs::Image::ConstPtr& image_msg);
    void configCallback(Config &config, uint32_t level);   
  };
}

#endif
