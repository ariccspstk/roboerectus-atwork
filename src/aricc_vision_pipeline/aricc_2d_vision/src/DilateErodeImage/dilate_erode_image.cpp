#include "aricc_2d_vision/dilate_erode_mask_image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>

namespace aricc_2d_vision
{
  void MorphologicalImageOperatorNodelet::onInit()
  {
    ConnectionBasedNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &MorphologicalImageOperatorNodelet::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
  }

  void MorphologicalImageOperatorNodelet::subscribe()
  {
    sub_ = pnh_->subscribe(
      "input", 1, &MorphologicalImageOperatorNodelet::imageCallback, this);
  }

  void MorphologicalImageOperatorNodelet::unsubscribe()
  {
    sub_.shutdown();
  }

  void MorphologicalImageOperatorNodelet::configCallback(
    Config &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    method_ = config.method;
    kernel_size_ = config.kernel_size;
    iterations_ = config.iterations;
  }
  
  void MorphologicalImageOperatorNodelet::imageCallback(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cv::Mat image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8)->image;
    int type;
    if (method_ == 0) {
      type = cv::MORPH_RECT;
    }
    else if (method_ == 1) {
      type = cv::MORPH_CROSS;
    }
    else if (method_ == 2) {
      type = cv::MORPH_ELLIPSE;
    }
    cv::Mat output_image;
    
    cv::Mat element = cv::getStructuringElement(
      type,
      cv::Size(2 * kernel_size_ + 1, 2 * kernel_size_+1),
      cv::Point(kernel_size_, kernel_size_));
    apply(image, output_image, element);
    pub_.publish(
      cv_bridge::CvImage(image_msg->header,
                         sensor_msgs::image_encodings::MONO8,
                         output_image).toImageMsg());
  }

  void DilateImage::apply(
    const cv::Mat& input, cv::Mat& output, const cv::Mat& element)
  {
    cv::dilate(input, output, element, /*anchor=*/cv::Point(-1,-1), iterations_);
  }

  void ErodeImage::apply(
    const cv::Mat& input, cv::Mat& output, const cv::Mat& element)
  {
    cv::erode(input, output, element, /*anchor=*/cv::Point(-1,-1), iterations_);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::DilateImage, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::ErodeImage, nodelet::Nodelet);
