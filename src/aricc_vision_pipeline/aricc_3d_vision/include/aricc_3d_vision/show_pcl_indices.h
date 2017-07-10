#ifndef ARICC_3D_VISION_BORDER_ESTIMATOR_H_
#define ARICC_3D_VISION_BORDER_ESTIMATOR_H_

#include <pcl_ros/pcl_nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include "aricc_3d_vision/pcl_conversion_util.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include <aricc_topic_tools/connection_based_nodelet.h>

namespace aricc_3d_vision
{
  class ShowPCLIndices: public aricc_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, PCLIndicesMsg> SyncPolicy;
  protected:
    virtual void onInit();
    virtual void execute(const sensor_msgs::PointCloud2::ConstPtr& msg,
                         const PCLIndicesMsg::ConstPtr& caminfo);
    
    virtual void subscribe();
    virtual void unsubscribe();
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_point_;
    message_filters::Subscriber<PCLIndicesMsg> sub_indices_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    ros::Publisher pub_cloud_;
    std::string color_;

  private:
    
  };
}

#endif
