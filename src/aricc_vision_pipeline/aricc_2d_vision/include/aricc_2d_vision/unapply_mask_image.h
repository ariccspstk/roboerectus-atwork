#ifndef ARICC_2D_VISION_UnAPPLY_MASK_IMAGE_H_
#define ARICC_2D_VISION_UNAPPLY_MASK_IMAGE_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace aricc_2d_vision
{
  class UnapplyMaskImage: public aricc_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::Image > ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image,
    sensor_msgs::Image > SyncPolicy;

  protected:

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void apply(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::Image::ConstPtr& mask_msg);

    bool approximate_sync_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::Image> sub_mask_;
    ros::Publisher pub_image_;
    
  private:
    
  };
}

#endif
