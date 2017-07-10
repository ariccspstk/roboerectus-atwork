#include "aricc_depth_vision/depth_to_gray_image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>


namespace aricc_depth_vision
{
  void DepthToGrayImage::onInit(){
    ConnectionBasedNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &DepthToGrayImage::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
  }

  void DepthToGrayImage::subscribe(){
    sub_ = pnh_->subscribe("input", 1, &DepthToGrayImage::execute, this);
  }

  void DepthToGrayImage::unsubscribe(){
    sub_.shutdown();
  }

  void DepthToGrayImage::configCallback(
    Config &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    min_range_  = config.min_range;
    max_range_  = config.max_range;
  }

  void DepthToGrayImage::execute(
    const sensor_msgs::Image::ConstPtr& image_msg){
    if(image_msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1){
      NODELET_ERROR("Only support depth image in 32FC1 encoding");
      return;
    }
    boost::mutex::scoped_lock lock(mutex_);
    cv::Mat input = cv_bridge::toCvShare(image_msg,
                                   image_msg->encoding)->image;
    cv::Mat output = cv::Mat::zeros(input.rows, input.cols, CV_8UC1);
    //Convert 32FC1 to 8UC1
    for(unsigned int j = 0; j < input.rows; ++j){
      for(unsigned int i = 0; i < input.cols; ++i){
          float d = input.at<float>(j,i);
          float p = 0.0;
          if( (d < min_range_) || (d > max_range_) ) p = 0.0;
          else 
            p = 255*(1.0-(d-min_range_)/(max_range_-min_range_));
          p = std::min( p, 255.0f );
          output.at<uchar>(j,i) =(unsigned char) p;
      }   
    }
    pub_.publish(
      cv_bridge::CvImage(
        image_msg->header,
        sensor_msgs::image_encodings::MONO8,
        output).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_depth_vision::DepthToGrayImage, nodelet::Nodelet);
