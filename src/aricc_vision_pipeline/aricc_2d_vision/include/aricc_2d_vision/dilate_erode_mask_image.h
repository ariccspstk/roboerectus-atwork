#ifndef ARICC_2D_VISION_DILATE_ERODE_MASK_IMAGE_H_
#define ARICC_2D_VISION_DILATE_ERODE_MASK_IMAGE_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include <aricc_2d_vision/MorphologicalMaskImageOperatorConfig.h>
#include <opencv2/opencv.hpp>

namespace aricc_2d_vision
{
  
  class MorphologicalImageOperatorNodelet:
    public aricc_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef aricc_2d_vision::MorphologicalMaskImageOperatorConfig Config;
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config &config, uint32_t level);
    virtual void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg);
    virtual void apply(const cv::Mat& input, cv::Mat& output, const cv::Mat& element) = 0;
    
    boost::mutex mutex_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    int method_;
    int kernel_size_;
    int iterations_;
  private:

  };
  
  class DilateImage: public MorphologicalImageOperatorNodelet
  {
  public:
  protected:
    virtual void apply(
      const cv::Mat& input, cv::Mat& output, const cv::Mat& element);
  };

  class ErodeImage: public MorphologicalImageOperatorNodelet
  {
  public:
  protected:
    virtual void apply(
      const cv::Mat& input, cv::Mat& output, const cv::Mat& element);
  };
}

#endif
