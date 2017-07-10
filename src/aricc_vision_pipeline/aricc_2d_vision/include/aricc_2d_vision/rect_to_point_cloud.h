#ifndef ARICC_2D_VISION_RECT_TO_POINT_CLOUD_H_
#define ARICC_2D_VISION_RECT_TO_POINT_CLOUD_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <aricc_vision_msgs/RotatedRectArray.h>
#include <sensor_msgs/PointCloud.h>

namespace aricc_2d_vision{
  class RectToPointCloud: public aricc_topic_tools::ConnectionBasedNodelet{
  public:
    
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void execute(
      const aricc_vision_msgs::RotatedRectArray::ConstPtr& rects_msg);

    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::string frame_;
  private:
    
  };
}

#endif
