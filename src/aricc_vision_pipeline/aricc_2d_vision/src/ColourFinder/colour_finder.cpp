#include "aricc_2d_vision/colour_finder.h"

namespace aricc_2d_vision{

  void ColourFinder::onInit(){
    ConnectionBasedNodelet::onInit();
    pnh_->param("h_high", h_high_, 10);
    pnh_->param("h_low",  h_low_, 10);

    pnh_->param("s_high", s_high_, 10);
    pnh_->param("s_low",  s_low_, 10);

    pnh_->param("v_high", v_high_, 10);
    pnh_->param("v_low",  v_low_, 10);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &ColourFinder::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
  }

  void ColourFinder::subscribe(){
    sub_ = pnh_->subscribe("input", 1, &ColourFinder::execute, this);
  }

  void ColourFinder::unsubscribe(){
    sub_.shutdown();
  }
  
  void ColourFinder::configCallback(
    Config &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    h_high_ = config.h_high;
    h_low_ = config.h_low;
    s_high_ = config.s_high;
    s_low_ = config.s_low;
    v_high_ = config.v_high;
    v_low_ = config.v_low;
  }

  void ColourFinder::execute(
    const sensor_msgs::Image::ConstPtr& image_msg){
    boost::mutex::scoped_lock lock(mutex_);
    
    //Convert input image to HSV colour space
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
      image_msg, image_msg->encoding);
    cv::Mat image = cv_ptr->image;
    cv::Mat hsv_image;

    if (image_msg->encoding == sensor_msgs::image_encodings::BGR8) {
      cv::cvtColor(image, hsv_image, CV_BGR2HSV);
    }
    else if (image_msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(image, hsv_image, CV_RGB2HSV);
    }
    else if (image_msg->encoding == sensor_msgs::image_encodings::BGRA8 ||
             image_msg->encoding == sensor_msgs::image_encodings::BGRA16) {
      cv::Mat tmp_image;
      cv::cvtColor(image, tmp_image, CV_BGRA2BGR);
      cv::cvtColor(tmp_image, hsv_image, CV_BGR2HSV);
    }
    else if (image_msg->encoding == sensor_msgs::image_encodings::RGBA8 ||
             image_msg->encoding == sensor_msgs::image_encodings::RGBA16) {
      cv::Mat tmp_image;
      cv::cvtColor(image, tmp_image, CV_RGBA2BGR);
      cv::cvtColor(tmp_image, hsv_image, CV_BGR2HSV);
    }
    else {
      NODELET_ERROR("unsupported format to HSV: %s", image_msg->encoding.c_str());
      return;
    }
    cv::Mat output = cv::Mat::zeros(hsv_image.rows, hsv_image.cols, CV_8UC1);
    //Generate Threshold Image
    cv::inRange(hsv_image, cv::Scalar(h_low_, s_low_, v_low_), 
      cv::Scalar(h_high_, s_high_, v_high_), output); 
    
    pub_.publish(
      cv_bridge::CvImage(
        image_msg->header,
        sensor_msgs::image_encodings::MONO8,
        output).toImageMsg());
  
  }
}
  
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::ColourFinder, nodelet::Nodelet);
