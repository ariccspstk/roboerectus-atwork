#include "aricc_2d_vision/rect_to_point_cloud.h"
#include "aricc_utils/geometry_utils.h"

namespace aricc_2d_vision{

  void RectToPointCloud::onInit(){
    ConnectionBasedNodelet::onInit();
    pnh_->param<std::string>("frame",  frame_, "test_link");
    pub_ = advertise<sensor_msgs::PointCloud>(*pnh_, "output", 1);
  }

  void RectToPointCloud::subscribe(){
    sub_ = pnh_->subscribe("input", 1, &RectToPointCloud::execute, this);

  }

  void RectToPointCloud::unsubscribe(){
    sub_.shutdown();
  }

  void RectToPointCloud::execute(
    const aricc_vision_msgs::RotatedRectArray::ConstPtr& rects_msg){
    sensor_msgs::PointCloud msg_point_cloud;
    geometry_msgs::Point32 p1, p2, p3, p4, p5;
    for(size_t i = 0; i < rects_msg->rects.size(); ++i){
      p5.x = rects_msg->rects.at(i).center.x;
      p5.y = rects_msg->rects.at(i).center.y;
      //p5.z = -rects_msg->rects.at(i).center.z;
      p5.z = 0.1;
      
      p1.x = rects_msg->rects.at(i).points.at(0).x;
      p1.y = rects_msg->rects.at(i).points.at(0).y;
      //p1.z = -rects_msg->rects.at(i).points.at(0).z;
      p1.z = 0.1;

      p2.x = rects_msg->rects.at(i).points.at(1).x;
      p2.y = rects_msg->rects.at(i).points.at(1).y;
      //p2.z = -rects_msg->rects.at(i).points.at(1).z;
      p2.z = 0.1;

      p3.x = rects_msg->rects.at(i).points.at(2).x;
      p3.y = rects_msg->rects.at(i).points.at(2).y;
      //p3.z = -rects_msg->rects.at(i).points.at(2).z;
      p3.z = 0.1;

      p4.x = rects_msg->rects.at(i).points.at(3).x;
      p4.y = rects_msg->rects.at(i).points.at(3).y;
      //p4.z = -rects_msg->rects.at(i).points.at(3).z;
      p4.z = 0.1;
      
      msg_point_cloud.points.push_back(p1);
      msg_point_cloud.points.push_back(p2);
      msg_point_cloud.points.push_back(p3);
      msg_point_cloud.points.push_back(p4);
      msg_point_cloud.points.push_back(p5);
    }
    msg_point_cloud.header = rects_msg->header;
    msg_point_cloud.header.frame_id = frame_;
    msg_point_cloud.header.stamp = ros::Time::now();
    pub_.publish( msg_point_cloud );
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::RectToPointCloud, nodelet::Nodelet);
