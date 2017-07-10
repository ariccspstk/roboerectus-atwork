#ifndef ARICC_2D_VISION_PROJECT_ROTATED_RECT_H_
#define ARICC_2D_VISION_PROJECT_ROTATED_RECT_H_

#include <aricc_vision_msgs/RotatedRectArray.h>
#include <aricc_vision_msgs/RotatedRect.h>
#include <aricc_2d_vision/ProjectRotatedRectConfig.h>
#include <aricc_topic_tools/connection_based_nodelet.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace aricc_2d_vision{
  class ProjectRotatedRect: public aricc_topic_tools::ConnectionBasedNodelet{
  public:
    typedef message_filters::sync_policies::ApproximateTime<
    aricc_vision_msgs::RotatedRectArray,
    sensor_msgs::CameraInfo > ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
    aricc_vision_msgs::RotatedRectArray,
    sensor_msgs::CameraInfo > SyncPolicy;

    typedef boost::shared_ptr<ProjectRotatedRect> Ptr;
    typedef ProjectRotatedRectConfig Config;
    
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void project(
      const aricc_vision_msgs::RotatedRectArray::ConstPtr& msg,
      const sensor_msgs::CameraInfo::ConstPtr& info_msg);
    virtual void configCallback(Config& config, uint32_t level);
    
    boost::mutex mutex_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    message_filters::Subscriber<aricc_vision_msgs::RotatedRectArray> sub_rects_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
    ros::Publisher pub_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    double z_;
    bool approximate_sync_;
    
  private:
    void getDim(aricc_vision_msgs::RotatedRect& rect);
    double getDist(geometry_msgs::Point p1, geometry_msgs::Point p2);
    
  };
}

#endif
