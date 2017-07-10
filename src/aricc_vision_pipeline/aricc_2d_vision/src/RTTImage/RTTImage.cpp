#include "aricc_2d_vision/RTTImage.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace aricc_2d_vision{

  void RTTImage::onInit(){
    ConnectionBasedNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &RTTImage::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<std_msgs::Header>(*pnh_, "output", 1);
    pnh_->param("threshold", threshold_, 1.0);
  }

  void RTTImage::subscribe(){
    sub_info_ = pnh_->subscribe("input", 1,
                                &RTTImage::infoCallback, this);
  }

  void RTTImage::unsubscribe(){
    sub_info_.shutdown();
  }

  void RTTImage::configCallback(
    Config &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    threshold_ = config.threshold;
  }
  
  bool RTTImage::processImage(cv::Mat image){
    //processing
    int count_white = 0;
    //Count white pixels in the input image 
    for( int y = 0; y < image.rows; y++){
	for( int x = 0; x < image.cols; x++){
      	  if ( image.at<cv::Vec3b>(y,x) == cv::Vec3b(255,255,255) ){
        	count_white++;
          } 
       }
    }
    //result
    int total = image.rows * image.cols; 
    double ratio = (double)count_white/(double)total;
    //NODELET_INFO("White pixels: %d",count_white );
    //NODELET_INFO("%d/%d = %lf",count_white, total, ratio );
    //NODELET_INFO("threshold = %lf", threshold_);

    if (ratio >= threshold_) return true;
    else
    return false;
  }

  void RTTImage::publishResult(){
    std_msgs::Header msg;
    msg.stamp = ros::Time::now();
    pub_.publish(msg);
  }
  
  void RTTImage::infoCallback(
    const sensor_msgs::Image::ConstPtr& msg){
      boost::mutex::scoped_lock lock(mutex_);	
      cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;
      bool isOk = processImage(image);
      if(isOk) publishResult();	
  }

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::RTTImage, nodelet::Nodelet);
