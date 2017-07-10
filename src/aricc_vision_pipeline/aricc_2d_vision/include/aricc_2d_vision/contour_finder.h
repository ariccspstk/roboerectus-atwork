#ifndef ARICC_2D_VISION_CONTOUR_FINDER_H_
#define ARICC_2D_VISION_CONTOUR_FINDER_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <aricc_vision_msgs/ContourArray.h>
#include <aricc_2d_vision/ContourFinderConfig.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace aricc_2d_vision{

  class ContourFinder: public aricc_topic_tools::ConnectionBasedNodelet{
  public:
    typedef aricc_2d_vision::ContourFinderConfig Config;

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
   
  private:
    boost::mutex mutex_;
    ros::Subscriber sub_;

    ros::Publisher pub_contours_;
    ros::Publisher pub_convex_contours_;
    ros::Publisher pub_poly_contours_;
    ros::Publisher pub_debug_image_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    
    double min_size_;
    double noise_size_;
    double min_dist_;
    int img_width_;
    int img_height_;
    bool debug_;
    bool draw_contours_;
    bool draw_convex_;
    bool draw_poly_;
    bool close_;
    double epsilon_;
    std::vector<std::vector<cv::Point> > contours_;
    std::vector<std::vector<cv::Point> > convex_contours_;
    std::vector<std::vector<cv::Point> > poly_contours_;
    std::vector<cv::Vec4i> hierarchy_;
 
    void execute(const sensor_msgs::Image::ConstPtr& image_msg);
    void executeColor(
      const sensor_msgs::Image::ConstPtr& mask_image_msg,
      const sensor_msgs::Image::ConstPtr& color_image_msg);
    void configCallback(Config &config, uint32_t level);   
    void mergeContours(std::vector<cv::Point> src,
                       std::vector<cv::Point>& dst);
    int onBorder(std::vector<cv::Point> contour, 
                  int width, int height, double dist);
    void pubResult(std_msgs::Header header);
  };
}

#endif
