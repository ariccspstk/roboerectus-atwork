#ifndef SOFTKINETICS_CAMERA_DRIVER_H_
#define SOFTKINETICS_CAMERA_DRIVER_H_

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <exception>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <time.h>

//ros include files
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <DepthSense.hxx>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>

using namespace DepthSense;

  class SoftKineticsCamera{
  public:
    SoftKineticsCamera();
    ~SoftKineticsCamera();
    void shutdown();
    bool start();

  protected:
  private:
    ros::NodeHandle pnh_;
    tf::TransformBroadcaster* transforms_;
    std::string base_link_;
    boost::shared_ptr<image_transport::ImageTransport> it_rgb_;
    boost::shared_ptr<image_transport::ImageTransport> it_depth_;
    image_transport::CameraPublisher pub_rgb_;
    image_transport::CameraPublisher pub_depth_;
    ros::Publisher pub_cloud_;
    boost::mutex connect_mutex_;

    sensor_msgs::Image rgb_image_msg_;
    sensor_msgs::Image depth_image_msg_;
    sensor_msgs::PointCloud2 cloud_msg_;
    sensor_msgs::CameraInfo rgb_camInfo_msg_;
    sensor_msgs::CameraInfo depth_camInfo_msg_;
    
    Context   g_context_;
    DepthNode g_dnode_;
    ColorNode g_cnode_;
    AudioNode g_anode_;
    std::vector<DepthSense::Device> devices_;
    StereoCameraParameters camera_params_;
    ProjectionHelper* g_pProjHelper_ = NULL;
    int confidence_threshold_;
    int depth_frame_rate_;
    int color_frame_rate_;
   
    bool initialized_; 
    bool g_bDeviceFound_;
    bool audio_enabled_;
    bool depth_enabled_;
    bool color_enabled_;
    bool cloud_enabled_;
    bool approximate_sync_;
    DepthSense::DepthNode::CameraMode depth_mode_;
    DepthSense::FrameFormat depth_frame_format_;
    DepthSense::CompressionType color_compression_;
    DepthSense::FrameFormat color_frame_format_;
    
    double timeStampToRosTime(uint64_t timestamp);
    void onNewAudioSample(AudioNode node, 
      AudioNode::NewSampleReceivedData data);
    template <typename T>
    T clip(const T& n, const T& lower, const T& upper);
    void YUV2RGB( DepthSense::Pointer<unsigned char> yuvData,
      unsigned char* rgbData, int width, int height);   
    void onNewColorSample(ColorNode node,
      ColorNode::NewSampleReceivedData data);
    void onNewDepthSample(DepthNode node,
      DepthNode::NewSampleReceivedData data);
    void configureAudioNode();
    void configureDepthNode();
    void configureColorNode();
    void configureNode(Node node);
    void onNodeConnected(Device device, Device::NodeAddedData data);
    void onNodeDisconnected(Device device, Device::NodeRemovedData data);
    void onDeviceConnected(Context context,
      Context::DeviceAddedData data);
    void onDeviceDisconnected(Context context,
      Context::DeviceRemovedData data);

  };
#endif
