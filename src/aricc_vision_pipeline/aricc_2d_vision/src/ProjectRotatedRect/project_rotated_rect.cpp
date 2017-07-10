#include "aricc_2d_vision/project_rotated_rect.h"

namespace aricc_2d_vision
{
  void ProjectRotatedRect::onInit()
  {
    ConnectionBasedNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("z", z_, 0.37);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&ProjectRotatedRect::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<aricc_vision_msgs::RotatedRectArray>(*pnh_, "output", 1);
  }

  void ProjectRotatedRect::subscribe(){
    sub_rects_.subscribe( *pnh_, "input/rects", 1 );
    sub_camera_info_.subscribe( *pnh_, "input/camera_info", 1 );

    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_rects_, sub_camera_info_);
      async_->registerCallback(boost::bind(&ProjectRotatedRect::project, this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_rects_, sub_camera_info_);
      sync_->registerCallback(boost::bind(&ProjectRotatedRect::project, this, _1, _2));
    }
  }

  void ProjectRotatedRect::unsubscribe()
  {
    sub_rects_.unsubscribe();
    sub_camera_info_.unsubscribe();
  }

  void ProjectRotatedRect::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    z_ = config.z;
  }

  double ProjectRotatedRect::getDist(geometry_msgs::Point p1,
                                     geometry_msgs::Point p2){
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    double dist = sqrt(dx*dx + dy*dy + dz*dz);
    return dist; 
  }

  void ProjectRotatedRect::getDim(aricc_vision_msgs::RotatedRect& rect){
    geometry_msgs::Point p0 = rect.points[0];
    geometry_msgs::Point p1 = rect.points[1];
    geometry_msgs::Point p2 = rect.points[2];

    double dist_1 = getDist(p0, p1);
    double dist_2 = getDist(p1, p2);
    if(dist_1 <= dist_2) {
      rect.width = dist_1;
      rect.height = dist_2;
      double dx = p2.x - p1.x;
      double dy = p2.y - p1.y;
      rect.angle = atan(dx/dy);
      
    }
    else{
      rect.width = dist_2;
      rect.height = dist_1;
      double dx = p1.x - p0.x;
      double dy = p1.y - p0.y;
      rect.angle = atan(dx/dy);
    }
  }  

  void ProjectRotatedRect::project(
    const aricc_vision_msgs::RotatedRectArray::ConstPtr& rects_msg,
    const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg){
    boost::mutex::scoped_lock lock(mutex_);
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(camera_info_msg);

    aricc_vision_msgs::RotatedRectArray projected_rects;
    cv::Point3d ray;
    for(size_t i = 0; i < rects_msg->rects.size(); ++i){
      //Porject center
      aricc_vision_msgs::RotatedRect projected_rect;
      ray = model.projectPixelTo3dRay(
        cv::Point2d(rects_msg->rects.at(i).center.x, rects_msg->rects.at(i).center.y));
      double alpha = z_ / ray.z;
      projected_rect.center.x = ray.x*alpha;
      projected_rect.center.y = ray.y*alpha;
      projected_rect.center.z = ray.z*alpha;
      //project vertices
      for(size_t j = 0; j < rects_msg->rects.at(i).points.size(); ++j){
        ray = model.projectPixelTo3dRay(
          cv::Point2d(rects_msg->rects.at(i).points.at(j).x, 
                      rects_msg->rects.at(i).points.at(j).y));
        geometry_msgs::Point p ;
        p.x = ray.x*alpha; 
        p.y = ray.y*alpha; 
        p.z = ray.z*alpha;
        projected_rect.points.push_back(p);
      }
      getDim(projected_rect);    
      projected_rects.rects.push_back(projected_rect);
    }
    projected_rects.header.seq = camera_info_msg->header.seq;
    projected_rects.header.frame_id = camera_info_msg->header.frame_id;
    projected_rects.header.stamp = ros::Time::now();
    pub_.publish(projected_rects);
    if (ray.z == 0.0) {
      NODELET_ERROR("Z value of projected ray is 0");
      return;
    }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::ProjectRotatedRect, nodelet::Nodelet);
