/*
 *****************************************************************
 *   ROS package name: aricc_3d_vision
 *   Author: Ian Wang
 *   Date of creation: March 2015
 *
 *****************************************************************
*/
#include "aricc_3d_vision/border_estimator.h"

namespace aricc_3d_vision
{
  BorderEstimator::BorderEstimator() : node_("~/border_estimator"){
    
    loop_rate_ = 20; //In Hz
    // planar or spherical
    node_.param("model_type", model_type_, std::string("planar"));
    node_.param("point_cloud_topic", point_cloud_topic_, std::string("input"));
    node_.param("camera_info_topic", camera_info_topic_, std::string("camera_info"));
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (node_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f = boost::bind (&BorderEstimator::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_border_ = node_.advertise<PCLIndicesMsg>("output_border_indices", 1);
    pub_veil_ = node_.advertise<PCLIndicesMsg>("output_veil_indices", 1);
    pub_shadow_ = node_.advertise<PCLIndicesMsg>("output_shadow_indices", 1);
    pub_range_image_ = node_.advertise<sensor_msgs::Image>("output_range_image", 1);
    pub_cloud_ = node_.advertise<sensor_msgs::PointCloud2>("output_cloud", 1);
    if (model_type_ == "planar"){
      sub_point_.subscribe(node_, point_cloud_topic_, 1);
      sub_camera_info_.subscribe(node_, camera_info_topic_, 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_point_, sub_camera_info_);
      sync_->registerCallback(boost::bind(&BorderEstimator::estimate, this, _1, _2));
    }
    else if (model_type_ == "laser") {
      sub_ = node_.subscribe(point_cloud_topic_, 1, &BorderEstimator::estimate, this);}
  }
  
  BorderEstimator::~BorderEstimator(){
    if (model_type_ == "planar") {
      sub_point_.unsubscribe();
      sub_camera_info_.unsubscribe();}
    else if (model_type_ == "laser") {
      sub_.shutdown();}
  }

  void BorderEstimator::publishCloud( ros::Publisher& pub,
                                      const pcl::PointIndices& inlier,
                                      const std_msgs::Header& header){
    PCLIndicesMsg msg;
    //msg.header = pcl_conversions::toPCL(header);
    msg.header = header;
    msg.indices = inlier.indices;
    pub.publish(msg);
  }

  void BorderEstimator::estimate( const sensor_msgs::PointCloud2::ConstPtr& msg){
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::fromROSMsg(*msg, cloudIn);
    pcl::RangeImage range_image;
    if (model_type_ == "sphere") {
      range_image = pcl::RangeImageSpherical();
    }
    range_image.createFromPointCloud( *cloud, 
                                      angular_resolution_,
                                      max_angle_width_, 
                                      max_angle_height_,
                                      Eigen::Affine3f::Identity(),
                                      pcl::RangeImage::CAMERA_FRAME,
                                      noise_level_,
                                      min_range_,
                                      border_size_);
    range_image.setUnseenToMaxRange();
    computeBorder(range_image, msg->header);
  }
  
  void BorderEstimator::estimate(
       const sensor_msgs::PointCloud2::ConstPtr& msg,
       const sensor_msgs::CameraInfo::ConstPtr& info){
    if (msg->height == 1) {
      ROS_ERROR("[BorderEstimator::estimate] pointcloud must be organized");
      return;
    }
    pcl::RangeImagePlanar range_image;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloudIn);
    pcl::fromROSMsg(*msg, cloud);
    Eigen::Affine3f dummytrans = Eigen::Affine3f::Identity();
    float fx = info->P[0];
    float cx = info->P[2];
    float tx = info->P[3];
    float fy = info->P[5];
    float cy = info->P[6];
    range_image.createFromPointCloudWithFixedSize (cloud,
                                                   msg->width,
                                                   msg->height,
                                                   cx, cy,
                                                   fx, fy,
                                                   dummytrans);
    range_image.setUnseenToMaxRange();
    computeBorder(range_image, msg->header);
  }

  void BorderEstimator::computeBorder( 
       const pcl::RangeImage& range_image,
       const std_msgs::Header& header){
    pcl::RangeImageBorderExtractor border_extractor (&range_image);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_extractor.compute (border_descriptions);
    pcl::PointIndices border_indices, veil_indices, shadow_indices;
    for (int y = 0; y < (int)range_image.height; ++y) {
      for (int x = 0; x < (int)range_image.width; ++x) {
        if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER]) {
          border_indices.indices.push_back (y*range_image.width + x);}
        if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT]) {
          veil_indices.indices.push_back (y*range_image.width + x);}
        if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER]) {
          shadow_indices.indices.push_back (y*range_image.width + x);}
      }
    }
    publishCloud(pub_border_, border_indices, header);
    publishCloud(pub_veil_, veil_indices, header);
    publishCloud(pub_shadow_, shadow_indices, header);
    cv::Mat image;
    rangeImageToCvMat(range_image, image);
    pub_range_image_.publish(
    cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,image).toImageMsg());
    // publish pointcloud
    //sensor_msgs::PointCloud2 ros_cloud;
    //pcl::toROSMsg(range_image, ros_cloud);
    //ros_cloud.header = header;
    //pub_cloud_.publish(ros_cloud);

    //Draw result and publish
    drawResult(cloudIn,  border_indices, GREEN);
    drawResult(cloudIn,  veil_indices, RED);
    drawResult(cloudIn,  shadow_indices, BLUE);
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloudIn, ros_cloud);
    pub_cloud_.publish(ros_cloud);
  }

  void BorderEstimator::drawResult(pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                                     pcl::PointIndices border_indices,
                                     myColor color ){
    uint32_t rgb;    
    if(color == RED) {
      uint8_t r = 255;
      uint8_t g = 0;
      uint8_t b = 0;
      rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    }
    else if(color == GREEN) {
      uint8_t r = 0;
      uint8_t g = 255;
      uint8_t b = 0;
      rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    }
    else if(color == BLUE) {
      uint8_t r = 0;
      uint8_t g = 0;
      uint8_t b = 255;
      rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    }
    for(unsigned int i = 0; i < border_indices.indices.size(); ++i ){
      cloud.points.at(i).rgb = *reinterpret_cast<float*>(&rgb);
    }
  }
  

  void BorderEstimator::configCallback(Config &config,uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    noise_level_ = config.noise_level;
    min_range_ = config.min_range;
    border_size_ = config.border_size;
    angular_resolution_ = config.angular_resolution;
    max_angle_height_ = config.max_angle_height;
    max_angle_width_ = config.max_angle_width;
  }

  void BorderEstimator::spin(){
    ros::Rate loop(loop_rate_);
    while(node_.ok()){
      ros::spinOnce();
      loop.sleep();
    }
  }
}
