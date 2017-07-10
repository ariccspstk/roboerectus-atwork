#include <aricc_2d_vision/mask_image_operator.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace aricc_2d_vision{
  void MaskImageOperator::onInit(){
    ConnectionBasedNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("method", method_, 0);
    pub_ = advertise<sensor_msgs::Image>(
      *pnh_, "output", 1);
  }

  void MaskImageOperator::subscribe(){
    sub_src1_.subscribe(*pnh_, "input/image1", 1);
    sub_src2_.subscribe(*pnh_, "input/image2", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproxSyncPolicy> >(100);
      async_->connectInput(sub_src1_, sub_src2_);
      async_->registerCallback(boost::bind(&MaskImageOperator::execute, this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_src1_, sub_src2_);
      sync_->registerCallback(boost::bind(&MaskImageOperator::execute, this, _1, _2));
    }
  }

  void MaskImageOperator::unsubscribe(){
    sub_src1_.unsubscribe();
    sub_src2_.unsubscribe();
  }

  void MaskImageOperator::execute(
    const sensor_msgs::Image::ConstPtr& src1_msg,
    const sensor_msgs::Image::ConstPtr& src2_msg){
    cv::Mat src1 = cv_bridge::toCvShare(
      src1_msg, src1_msg->encoding)->image;
    cv::Mat src2 = cv_bridge::toCvShare(
      src2_msg, src2_msg->encoding)->image;
    cv::Mat result;
    if(method_ == 0) cv::add(src1, src2, result);
    if(method_ == 1) cv::bitwise_and(src1, src2, result);
    if(method_ == 2) cv::bitwise_or(src1, src2, result);
    if(method_ == 3) cv::bitwise_xor(src1, src2, result);
    pub_.publish(
      cv_bridge::CvImage(src1_msg->header,
                         sensor_msgs::image_encodings::MONO8,
                         result).toImageMsg());
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::MaskImageOperator, nodelet::Nodelet);
