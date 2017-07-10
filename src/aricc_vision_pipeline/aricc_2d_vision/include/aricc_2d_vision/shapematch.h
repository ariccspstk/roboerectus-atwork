#ifndef ARICC_2D_VISION_SHAPEMATCH_H_
#define ARICC_2D_VISION_SHAPEMATCH_H_

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <aricc_topic_tools/connection_based_nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <aricc_vision_msgs/ContourArray.h>
#include <aricc_2d_vision/ShapeMatchConfig.h>



namespace aricc_2d_vision{
   
  class ShapeMatch: public aricc_topic_tools::ConnectionBasedNodelet{
  public:
    typedef aricc_2d_vision::ShapeMatchConfig Config;
    typedef message_filters::sync_policies::ExactTime<
    aricc_vision_msgs::ContourArray,
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo > SyncPolicy;

    typedef message_filters::sync_policies::ApproximateTime<
    aricc_vision_msgs::ContourArray,
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo > ApproxSyncPolicy;



  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();


  private:
    boost::mutex mutex_;
    message_filters::Subscriber<aricc_vision_msgs::ContourArray> sub_contour_;
    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_image_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy> > async_;
    //boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;

    bool debug_;
    bool approximate_sync_;

   
    //void configCallback(Config& config, uint32_t level);
    bool loadReferences();
    void execute(
      const aricc_vision_msgs::ContourArray::ConstPtr& msg,
      const sensor_msgs::Image::ConstPtr& image_msg);
    struct Reference{
      std::string name;
    };
    void configCallback(
      Config &config, uint32_t level);

  };
    
}

#endif
