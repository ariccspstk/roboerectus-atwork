#ifndef ARICC_DEPTH_VISION_DEPTH_REGISTER_IMAGE_H_
#define ARICC_DEPTH_VISION_DEPTH_REGISTER_IMAGE_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/image_encodings.h>
#include <aricc_depth_vision/depth_traits.h>

namespace aricc_depth_vision{

  class DepthRegister: public aricc_topic_tools::ConnectionBasedNodelet{
  
  public:
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      sensor_msgs::CameraInfo,
      sensor_msgs::CameraInfo > ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      sensor_msgs::CameraInfo,
      sensor_msgs::CameraInfo > SyncPolicy;
  
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void execute(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::CameraInfo::ConstPtr& depth_info_msg,
      const sensor_msgs::CameraInfo::ConstPtr& rgb_info_msg);
    template<typename T> void convert(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::ImagePtr& registered_msg,
      const Eigen::Affine3d& depth_to_rgb);

    bool approximate_sync_;
    boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_;

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_depth_info_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_rgb_info_;

    ros::Publisher pub_image_;
    ros::Publisher pub_image_info_;
    image_geometry::PinholeCameraModel depth_model_, rgb_model_;
    boost::mutex mutex_;
    int queue_size_;

  private:

  };
}

#endif
