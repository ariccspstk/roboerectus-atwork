#include "aricc_2d_vision/edge_finder.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>


namespace aricc_2d_vision
{
  void EdgeFinder::onInit(){
    ConnectionBasedNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &EdgeFinder::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
  }

  void EdgeFinder::subscribe(){
    sub_ = pnh_->subscribe("input", 1, &EdgeFinder::execute, this);
  }

  void EdgeFinder::unsubscribe(){
    sub_.shutdown();
  }

  void EdgeFinder::configCallback(
    Config &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    low_threshold_ = config.low_threshold;
    ratio_ = config.ratio;
    kernel_size_ = config.kernel_size;
    L2_gradient_ = config.L2_gradient;
  }

  void EdgeFinder::execute(
    const sensor_msgs::Image::ConstPtr& image_msg){
    boost::mutex::scoped_lock lock(mutex_);
    cv::Mat input, output;
    try{
      input = cv_bridge::toCvCopy(
        image_msg, image_msg->encoding)->image;
      output = cv::Mat::zeros(input.rows, input.cols, CV_8UC1);
      if (image_msg->encoding == sensor_msgs::image_encodings::BGR8 ||
          image_msg->encoding == sensor_msgs::image_encodings::RGB8){
        cv::cvtColor(input, output, CV_BGR2GRAY);
      }
      else output = input;
      
      if(!(kernel_size_%2)) kernel_size_++;
      cv::Canny(output, output, low_threshold_, 
              low_threshold_*ratio_, kernel_size_, L2_gradient_);
    }
    catch(cv::Exception &e){
      NODELET_ERROR("Image processing error: %s %s %s %i", 
                     e.err.c_str(), e.func.c_str(), 
                     e.file.c_str(), e.line);
    }
    pub_.publish(cv_bridge::CvImage(
                               image_msg->header,
                               sensor_msgs::image_encodings::MONO8,
                               output).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::EdgeFinder, nodelet::Nodelet);
