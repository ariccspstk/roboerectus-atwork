#ifndef ARICC_2D_VISION_ASAD_VISION_H_
#define ARICC_2D_VISION_ASAD_VISION_H_

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

  class ObjectDetector: public aricc_topic_tools::ConnectionBasedNodelet{
  public:
    enum haar_sample_type{POSITIVE, NEGATIVE};
    enum get_contour_type{THRESHOLD, CANNY};
    enum remove_background{BY_DISTANCE, BY_COLOR};
    enum object_symbolic_description{F20_20_B, F20_20_G, S40_40_B, S40_40_G, M20_100, M20, M30, R20, BEARING_BOX, BEARING, AXIS, DISTANCE_TUBE, MOTOR};
    enum object_orientation{VERTICAL, HORIZONTAL, TILTED, UP};

    struct objData
    {
        object_symbolic_description objDescription;
        object_orientation objOrientation;
        int objX;
        int objY;
        double radius;
    };

    float dist;
    cv::Vec3b objColor;

    //ObjectDetector(cv::Mat cSrc, cv::Mat dSrc) : colorImage(cSrc.clone()), depthImage(dSrc.clone()) {}

    void process (cv::Mat);
    cv::Mat quantize (cv::Mat, bool);
    void saveImage(cv::Mat, haar_sample_type, int);
    void saveImage(cv::Mat, int);
    void getContours(get_contour_type, bool);
    std::vector<std::vector<cv::Point> > getContours(cv::Mat);
    bool compareObjects(cv::Mat);
    cv::Mat selectedColores(cv::Mat, bool);
    void removeBG(cv::Mat, cv::Mat, remove_background);
    float getDistance(cv::Mat depthImage, int x, int y);
    cv::Vec3b getColor(cv::Mat input, int x, int y);
    cv::Mat adjustLight(cv::Mat input);

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
   
  private:
    cv::Mat colorImage;
    cv::Mat depthImage;

    boost::mutex mutex_;
    ros::Subscriber sub_;

    int img_width_;
    int img_height_;


    ros::Publisher pub_contours_;
    ros::Publisher pub_convex_contours_;
    ros::Publisher pub_poly_contours_;
    ros::Publisher pub_debug_image_;
    //boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    
    std::vector<std::vector<cv::Point> > contours_;
    std::vector<std::vector<cv::Point> > convex_contours_;
    std::vector<std::vector<cv::Point> > poly_contours_;
    std::vector<cv::Vec4i> hierarchy_;
 
    void execute(const sensor_msgs::Image::ConstPtr& image_msg);
    void executeColor(
      const sensor_msgs::Image::ConstPtr& mask_image_msg,
      const sensor_msgs::Image::ConstPtr& color_image_msg);
    //void configCallback(Config &config, uint32_t level);   
    void mergeContours(std::vector<cv::Point> src,
                       std::vector<cv::Point>& dst);
    int onBorder(std::vector<cv::Point> contour, 
                  int width, int height, double dist);
    void pubResult(std_msgs::Header header);
  };
}

#endif
