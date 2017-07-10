#include "aricc_2d_vision/create_threshold_image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>


namespace aricc_2d_vision
{
  void CreateThresholdImage::onInit(){
    ConnectionBasedNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &CreateThresholdImage::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
  }

  void CreateThresholdImage::subscribe(){
    sub_ = pnh_->subscribe("input", 1, &CreateThresholdImage::execute, this);
  }

  void CreateThresholdImage::unsubscribe(){
    sub_.shutdown();
  }

  void CreateThresholdImage::configCallback(
    Config &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    block_size_ = config.block_size;
    constant_   = config.constant;
    method_     = config.method;
    value_      = config.value;
    type_       = config.type;
  }

  void CreateThresholdImage::execute(
    const sensor_msgs::Image::ConstPtr& image_msg){
    boost::mutex::scoped_lock lock(mutex_);
    cv::Mat input;
    //Input is the color image
    if (image_msg->encoding == sensor_msgs::image_encodings::BGR8 ||
        image_msg->encoding == sensor_msgs::image_encodings::RGB8){
      input = cv_bridge::toCvCopy(
        image_msg, sensor_msgs::image_encodings::MONO8)->image;
    }
    else if(image_msg->encoding == sensor_msgs::image_encodings::MONO8){
      input = cv_bridge::toCvShare(image_msg,
                                   image_msg->encoding)->image;
    }
    else {
      NODELET_ERROR("Wrong image encoding, only support RGB8 and MONO8");     
      return;
    }
    cv::Mat output = cv::Mat::zeros(input.rows, input.cols, CV_8UC1);

    int method = 0, type = 0;
    if(type_ == 0) type = CV_THRESH_BINARY_INV;
    else if(type == 1) type = CV_THRESH_BINARY;
 
    if(method_ == 2){
      cv::threshold( input, output, value_, 255, type_ );
    }
    else{
      if(method_ == 0) method = cv::ADAPTIVE_THRESH_MEAN_C;
      else if(method_ == 1) method = cv::ADAPTIVE_THRESH_GAUSSIAN_C;
      if(!(block_size_%2)) block_size_++;
      block_size_ = std::max(block_size_, (unsigned int)3);
      adaptiveThreshold(input, output, 255, method, type, block_size_, constant_);
    }
     
    pub_.publish(
      cv_bridge::CvImage(
        image_msg->header,
        sensor_msgs::image_encodings::MONO8,
        output).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::CreateThresholdImage, nodelet::Nodelet);
