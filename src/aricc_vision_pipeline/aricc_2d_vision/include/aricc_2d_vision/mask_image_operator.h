#ifndef ARICC_2D_VISION_MASK_IMAGE_OPERATOR_H_
#define ARICC_2D_VISION_MASK_IMAGE_OPERATOR_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace aricc_2d_vision{
  class MaskImageOperator: public aricc_topic_tools::ConnectionBasedNodelet{
  public:
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image,
    sensor_msgs::Image > SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::Image > ApproxSyncPolicy;
    
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void execute(
      const sensor_msgs::Image::ConstPtr& src1_msg,
      const sensor_msgs::Image::ConstPtr& src2_msg);

    bool approximate_sync_;
    int method_;
    ros::Publisher pub_;
    message_filters::Subscriber<sensor_msgs::Image> sub_src1_;
    message_filters::Subscriber<sensor_msgs::Image> sub_src2_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy> > async_;
  private:
    
  };
}

#endif
