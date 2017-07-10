#include "aricc_2d_vision/unapply_mask_image.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "aricc_2d_vision/image_utils.h"

namespace aricc_2d_vision
{
  void UnapplyMaskImage::onInit()
  {
    ConnectionBasedNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pub_image_ = advertise<sensor_msgs::Image>(
      *pnh_, "output", 1);
  }

  void UnapplyMaskImage::subscribe()
  {
    sub_image_.subscribe(*pnh_, "input", 1);
    sub_mask_.subscribe(*pnh_, "input/mask", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_image_, sub_mask_);
      async_->registerCallback(boost::bind(&UnapplyMaskImage::apply, this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_image_, sub_mask_);
      sync_->registerCallback(boost::bind(&UnapplyMaskImage::apply, this, _1, _2));
    }
  }

  void UnapplyMaskImage::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_mask_.unsubscribe();
  }

  void UnapplyMaskImage::apply(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::Image::ConstPtr& mask_msg)
  {
    cv::Mat image = cv_bridge::toCvShare(image_msg,
                                         image_msg->encoding)->image;
    cv::Mat mask = cv_bridge::toCvShare(mask_msg,
                                        mask_msg->encoding)->image;
    cv::Mat output;
    bool single_channel = false;
    if (image_msg->encoding == sensor_msgs::image_encodings::BGR8 ||
        image_msg->encoding == sensor_msgs::image_encodings::RGB8) {
      single_channel = false;
      output = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
    }
    else if(image_msg->encoding == sensor_msgs::image_encodings::MONO8){
      single_channel = true;
      output = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC1);
    }
    
    cv::Rect region = boundingRectOfMaskImage(mask);
    //NODELET_INFO("Region:%d,%d; image:%d,%d",region.x, region.y, image.rows, image. cols);
    for (int j = 0; j < image.rows; j++) {
      for (int i = 0; i < image.cols; i++) {
        if(single_channel) {
          //NODELET_INFO("%d,%d",j+region.y, i+region.x);
          output.at<uchar>(j + region.y, i + region.x)
              = image.at<uchar>(j, i);
        }
        else{
          //NODELET_INFO("%d,%d",j+region.y, i+region.x);
          output.at<cv::Vec3b>(j + region.y, i + region.x)
              = image.at<cv::Vec3b>(j, i);
        }
      }
    }
    pub_image_.publish(cv_bridge::CvImage(
                         image_msg->header,
                         image_msg->encoding,
                         output).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::UnapplyMaskImage, nodelet::Nodelet);
