#ifndef ARICC_3D_VISION_BORDER_ESTIMATOR_H_
#define ARICC_3D_VISION_BORDER_ESTIMATOR_H_

#include <pcl_ros/pcl_nodelet.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include "aricc_3d_vision/pcl_conversion_util.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include <aricc_topic_tools/connection_based_nodelet.h>

#include <aricc_3d_vision/BorderEstimatorConfig.h>
#include <dynamic_reconfigure/server.h>

namespace aricc_3d_vision
{
  class BorderEstimator: public aricc_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> SyncPolicy;
    typedef BorderEstimatorConfig Config;
  protected:
    virtual void onInit();
    virtual void estimate(const sensor_msgs::PointCloud2::ConstPtr& msg,
                          const sensor_msgs::CameraInfo::ConstPtr& caminfo);
    virtual void estimate(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void computeBorder(
      const pcl::RangeImage& image,
      const std_msgs::Header& header);
    virtual void publishCloud(ros::Publisher& pub,
                              const pcl::PointIndices& inlier,
                              const std_msgs::Header& header);
    
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config &config, uint32_t level);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_point_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    ros::Publisher pub_border_, pub_veil_, pub_shadow_;
    ros::Publisher pub_range_image_;
    ros::Publisher pub_cloud_;
    ros::Subscriber sub_;
    std::string model_type_;
    boost::mutex mutex_;
    double noise_level_;
    double min_range_;
    int border_size_;
    double angular_resolution_;
    double max_angle_height_;
    double max_angle_width_;

  private:
    
  };
}

#endif
