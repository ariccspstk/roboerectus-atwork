#include "aricc_2d_vision/rect_to_mask_image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace aricc_2d_vision{

  void RectToMaskImage::onInit(){
    ConnectionBasedNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &RectToMaskImage::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    pnh_->param("project", project_, false);
    pnh_->param("z", z_, 1.0);
  }

  void RectToMaskImage::subscribe(){
    sub_info_ = pnh_->subscribe("input/camera_info", 1,
                                &RectToMaskImage::infoCallback, this);
  }

  void RectToMaskImage::unsubscribe(){
    sub_info_.shutdown();
  }

  void RectToMaskImage::configCallback(
    Config &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    left_top_x_ = config.left_top_x;
    left_top_y_ = config.left_top_y;
    width_      = config.width;
    height_     = config.height;
    project_    = config.project;
    z_          = config.z;
  }

  void RectToMaskImage::createProjectMask(cv::Mat& mask_image,
    sensor_msgs::CameraInfo info_msg){
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(info_msg);
    /*OpenCV coordinate ---->x
                        |
                        |
                        v y
    */

    cv::Point3d p_3d_f, p_3d_s;
    cv::Point2d p_2d_f, p_2d_s;
    
    p_3d_f.x = left_top_x_;
    p_3d_f.y = left_top_y_;
    p_3d_f.z = z_;
    p_3d_s.x = left_top_x_ + width_;
    p_3d_s.y = left_top_y_ + height_;
    p_3d_s.z = z_;

    p_2d_f = model.project3dToPixel(p_3d_f);
    p_2d_s = model.project3dToPixel(p_3d_s);

    double width   = p_2d_s.x - p_2d_f.x;
    double height  = p_2d_s.y - p_2d_f.y;
    double min_x  = std::max(p_2d_f.x, 0.0);
    double min_y  = std::max(p_2d_f.y, 0.0);
    //NODELET_INFO("Region:[%.2lf, %.2lf, %.2lf, %.2lf]:",
    //  min_x, min_y, width, height);
    width  = std::min(width,  (info_msg.width - min_x));
    height = std::min(height, (info_msg.height - min_y));

    //NODELET_INFO("Point_1:[%.2lf, %.2lf, %.2lf]:",
    //  p_3d_f.x, p_3d_f.y, p_3d_f.z);
    //NODELET_INFO("Point_2:[%.2lf, %.2lf, %.2lf]:",
    //  p_3d_s.x, p_3d_s.y, p_3d_s.z);
    //NODELET_INFO("Region:[%.2lf, %.2lf, %.2lf, %.2lf]:",
    //  min_x, min_y, width, height);

    cv::Rect region((int)min_x, (int)min_y, (int)width, (int)height);
    cv::rectangle(mask_image, region, cv::Scalar(255), CV_FILLED);
    pub_.publish(cv_bridge::CvImage(
                     info_msg.header,
                     sensor_msgs::image_encodings::MONO8,
                     mask_image).toImageMsg());
  
  }
  
  void RectToMaskImage::createMask(cv::Mat& mask_image,
    sensor_msgs::CameraInfo info_msg){
    double min_x  = std::max(left_top_x_, 0.0);
    double min_y  = std::max(left_top_y_, 0.0);
    double width  = std::min(width_,  (info_msg.width - min_x));
    double height = std::min(height_, (info_msg.height - min_y));
    
    cv::Rect region((int)min_x, (int)min_y, (int)width, (int)height);
    cv::rectangle(mask_image, region, cv::Scalar(255), CV_FILLED);
    pub_.publish(cv_bridge::CvImage(
                     info_msg.header,
                     sensor_msgs::image_encodings::MONO8,
                     mask_image).toImageMsg());
  }
  
  void RectToMaskImage::infoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg){
    boost::mutex::scoped_lock lock(mutex_);
    if (info_msg) {
      cv::Mat mask_image = cv::Mat::zeros(info_msg->height,
                                          info_msg->width,
                                          CV_8UC1);
      if(project_) createProjectMask(mask_image, *info_msg);
      else createMask(mask_image, *info_msg);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::RectToMaskImage, nodelet::Nodelet);
