#include "aricc_2d_vision/smoothing_image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>


namespace aricc_2d_vision
{
  void SmoothingImage::onInit(){
    ConnectionBasedNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &SmoothingImage::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
  }

  void SmoothingImage::subscribe(){
    sub_ = pnh_->subscribe("input", 1, &SmoothingImage::execute, this);
  }

  void SmoothingImage::unsubscribe(){
    sub_.shutdown();
  }

  void SmoothingImage::configCallback(
    Config &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    method_ = config.method;
    kernel_size_ = config.kernel_size;
  }

  void SmoothingImage::execute(
    const sensor_msgs::Image::ConstPtr& image_msg){
    boost::mutex::scoped_lock lock(mutex_);
    if(image_msg->encoding != sensor_msgs::image_encodings::RGB8 &&
       image_msg->encoding != sensor_msgs::image_encodings::BGR8 &&
       image_msg->encoding != sensor_msgs::image_encodings::MONO8 ){
      NODELET_ERROR("Wrong image format:%s, only support %s and %s, and %s", 
        image_msg->encoding.c_str(), 
        sensor_msgs::image_encodings::RGB8.c_str(),
        sensor_msgs::image_encodings::BGR8.c_str(),
        sensor_msgs::image_encodings::MONO8.c_str());
      return;
    }
    cv::Mat output = cv_bridge::toCvCopy(
      image_msg, image_msg->encoding)->image;
    if(!(kernel_size_%2)) kernel_size_++;
    if(method_ == 0){
      cv::blur( output, output, cv::Size( kernel_size_, kernel_size_ ), 
                cv::Point(-1,-1) );
    }
    else if(method_ == 1){
      cv::GaussianBlur( output, output, cv::Size( kernel_size_, kernel_size_ ), 
                        0, 0 );
    }
    else if(method_ == 2){
      cv::medianBlur( output, output, kernel_size_ );
    }
    
   
    pub_.publish(cv_bridge::CvImage(
                               image_msg->header,
                               image_msg->encoding,
                               output).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::SmoothingImage, nodelet::Nodelet);
