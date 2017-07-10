/*
 *****************************************************************
 *   ROS package name: aricc_3d_vision
 *   Author: Ian Wang
 *   Date of creation: March 2015
 *
 *****************************************************************
*/
#define BOOST_PARAMETER_MAX_ARITY 7
#include "aricc_3d_vision/pcl_conversion_util.h"
#include "aricc_3d_vision/show_pcl_indices.h"
#include <cv_bridge/cv_bridge.h>

namespace aricc_3d_vision
{
  void ShowPCLIndices::onInit()
  {
    ConnectionBasedNodelet::onInit();
    pnh_->param("color", color_, std::string("red"));
    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_cloud", 1);
  }

  void ShowPCLIndices::subscribe(){
    sub_indices_.subscribe(*pnh_, "input_indices", 1);
    sub_point_.subscribe(*pnh_, "input_cloud", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_point_, sub_indices_);
    sync_->registerCallback(boost::bind(&ShowPCLIndices::execute, this, _1, _2));
  }

  void ShowPCLIndices::unsubscribe(){
    sub_indices_.unsubscribe();
    sub_point_.unsubscribe();
  }

  void ShowPCLIndices::execute( 
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    const PCLIndicesMsg::ConstPtr& ind){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    uint32_t rgb;
    if(color_ == "red"){
      uint8_t r = 255;
      uint8_t g = 0;
      uint8_t b = 0;
      rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    }
    else if(color_ == "green"){
      uint8_t r = 0;
      uint8_t g = 255;
      uint8_t b = 0;
      rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    }
    else if(color_ == "blue"){
      uint8_t r = 0;
      uint8_t g = 0;
      uint8_t b = 255;
      rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    }
    for(unsigned int i = 0; i < ind->indices.size(); ++i ){
      pcl_cloud->points.at(i).rgb = *reinterpret_cast<float*>(&rgb);
    }
    // publish pointcloud
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*pcl_cloud, ros_cloud);
    ros_cloud.header = msg->header;
    pub_cloud_.publish(ros_cloud);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_3d_vision::ShowPCLIndices,
                        nodelet::Nodelet);
