#include "aricc_2d_vision/apply_mask_image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "aricc_2d_vision/image_utils.h"

namespace aricc_2d_vision
{
  void ApplyMaskImage::onInit()
  {
    ConnectionBasedNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pub_image_ = advertise<sensor_msgs::Image>(
      *pnh_, "output", 1);
    pub_mask_ = advertise<sensor_msgs::Image>(
      *pnh_, "output/mask", 1);
  }

  void ApplyMaskImage::subscribe()
  {
    sub_image_.subscribe(*pnh_, "input", 1);
    sub_mask_.subscribe(*pnh_, "input/mask", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_image_, sub_mask_);
      async_->registerCallback(boost::bind(&ApplyMaskImage::apply, this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_image_, sub_mask_);
      sync_->registerCallback(boost::bind(&ApplyMaskImage::apply, this, _1, _2));
    }
  }

  void ApplyMaskImage::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_mask_.unsubscribe();
  }

  void ApplyMaskImage::apply(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::Image::ConstPtr& mask_msg)
  {
    cv::Mat image = cv_bridge::toCvShare(image_msg,
                                         image_msg->encoding)->image;
    cv::Mat mask = cv_bridge::toCvShare(mask_msg,
                                        mask_msg->encoding)->image;
    if (image.cols != mask.cols || image.rows != mask.rows) {
      NODELET_ERROR("size of image and mask is different");
      NODELET_ERROR("image: %dx%dx", image.cols, image.rows);
      NODELET_ERROR("mask: %dx%dx", mask.cols, mask.rows);
      return;
    }
    
    cv::Rect region = boundingRectOfMaskImage(mask);
    cv::Mat clipped_mask = mask(region);
    pub_mask_.publish(cv_bridge::CvImage(
                        mask_msg->header,
                        mask_msg->encoding,
                        clipped_mask).toImageMsg());

    cv::Mat clipped_image = image(region);
    cv::Mat masked_image;
    clipped_image.copyTo(masked_image, clipped_mask);
    pub_image_.publish(cv_bridge::CvImage(
                         image_msg->header,
                         image_msg->encoding,
                         masked_image).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::ApplyMaskImage, nodelet::Nodelet);
