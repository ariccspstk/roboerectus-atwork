#include "aricc_2d_vision/rect_based_object_detection.h"
#include "aricc_utils/geometry_utils.h"

namespace aricc_2d_vision{

  void RectBasedObjectDetection::onInit(){
    ConnectionBasedNodelet::onInit();
    pnh_->param("width_tolerance",  width_tolerance_, 0.001);
    pnh_->param("height_tolerance",  height_tolerance_, 0.001);
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("color_h_tol", color_h_tol_, 20.0);
    pnh_->param("color_s_tol", color_s_tol_, 20.0);
    pnh_->param("color_v_tol", color_v_tol_, 20.0);
    pnh_->param("density_tolerance", density_tolerance_, 20.0);
    pnh_->param("debug", debug_, false);
    pnh_->param("print_info", print_info_, false);
    pnh_->param<std::string>("list_title", list_title_, "objects");

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&RectBasedObjectDetection::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<aricc_vision_msgs::ObjectArray>(*pnh_, "output", 1);
    if(debug_){
      pub_debug_image_ = advertise<sensor_msgs::Image>(*pnh_, "debug", 1);
    }
    loadObjects();
  }

  void RectBasedObjectDetection::subscribe(){
    sub_rects_.subscribe(*pnh_,"input/rects", 1);
    sub_image_.subscribe(*pnh_,"input/image", 1);
    sub_image_info_.subscribe(*pnh_,"input/image_info", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproxSyncPolicy> >(100);
      async_->connectInput(sub_rects_, sub_image_, sub_image_info_);
      async_->registerCallback(boost::bind(&RectBasedObjectDetection::execute, this, _1, _2, _3));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_rects_, sub_image_,sub_image_info_);
      sync_->registerCallback(boost::bind(&RectBasedObjectDetection::execute, this, _1, _2, _3));
    }
  }

  void RectBasedObjectDetection::unsubscribe(){
    sub_rects_.unsubscribe();
    sub_image_.unsubscribe();
    sub_image_info_.unsubscribe();
  }

  void RectBasedObjectDetection::configCallback(Config& config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    width_tolerance_   = config.width_tolerance;
    height_tolerance_  = config.height_tolerance;
    density_tolerance_ = config.density_tolerance;
    color_h_tol_       = config.color_h_tol;
    color_s_tol_       = config.color_s_tol;
    color_v_tol_       = config.color_v_tol;
    debug_             = config.debug;
    print_info_        = config.print_info;
  }

  void RectBasedObjectDetection::loadObjects(){
    object_list_.clear();
    //Loading tasks from config files
    NODELET_INFO("----------");
    NODELET_INFO("Loading %s ...", list_title_.c_str());
    XmlRpc::XmlRpcValue object_list;

    if( pnh_->getParam( list_title_, object_list) ){
      NODELET_INFO("Found %s list, size: %d",
        list_title_.c_str(),object_list.size());
      ROS_ASSERT(object_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      if( object_list.getType() == XmlRpc::XmlRpcValue::TypeArray ){
        for( size_t i = 0; i < object_list.size(); ++i){
          std::string name =
          std::string(object_list[i]["name"]);
          std::string width =
          std::string(object_list[i]["width"]);
          std::string height =
          std::string(object_list[i]["height"]);
          std::string color_h =
          std::string(object_list[i]["color_h"]);
          std::string color_s =
          std::string(object_list[i]["color_s"]);
          std::string color_v =
          std::string(object_list[i]["color_v"]);
          std::string density =
          std::string(object_list[i]["density"]);
          NODELET_INFO("%s: %s,%s,%s,%s,%s,%s", 
            name.c_str(), width.c_str(), height.c_str(), 
            color_h.c_str(),color_s.c_str(), color_v.c_str(),
            density.c_str());
          Object obj;
          obj.name    = name;
          obj.width   = atof(width.c_str());
          obj.height  = atof(height.c_str());
          obj.color_h = atof(color_h.c_str());
          obj.color_s = atof(color_s.c_str());
          obj.color_v = atof(color_v.c_str());
          obj.density = atof(density.c_str());
          object_list_.push_back(obj);
        }
        NODELET_INFO("Loaded %lu %s",object_list_.size(), list_title_.c_str());
      }
    }
    else NODELET_ERROR("Couldn`t load %s", list_title_.c_str());
    NODELET_INFO("----------");
  }

  std::string RectBasedObjectDetection::detect( 
    aricc_vision_msgs::RotatedRect rect ){
    double width    = rect.width;
    double height   = rect.height;
    double color_h  = rect.color_h;
    double color_s  = rect.color_s;
    double color_v  = rect.color_v;
    double density  = rect.density;
    double angle    = aricc_utils::rad2Deg(rect.angle);
    geometry_msgs::Point center = rect.center;
    std::vector <Object>::iterator it = object_list_.begin();
    
    for(; it!= object_list_.end(); ++it){
      double diff_width   = fabs( width - it->width );
      double diff_height  = fabs( height - it->height );
      double diff_color_h = fabs( color_h - it->color_h );
      double diff_color_s = fabs( color_s - it->color_s );
      double diff_color_v = fabs( color_v - it->color_v );
      double diff_density = fabs( density - it->density );
      if(print_info_){
        NODELET_INFO("Object:[%.3lf,%.3lf],[%.3lf,%.3lf, %.3lf],%.3lf,[%.3lf,%.3lf,%.3lf]",width, height, color_h, color_s, color_v, density, center.x, center.y, angle);
        NODELET_INFO("Ref: %s,[%.3lf,%.3lf],[%.3lf,%.3lf, %.3lf], %.3lf",
          it->name.c_str(),it->width, it->height, it->color_h, it->color_s, it->color_v,it->density);
        NODELET_INFO("Diff:[%.3lf,%.3lf],[%.3lf, %.3lf, %.3lf], %.3lf",
          diff_width, diff_height, diff_color_h, diff_color_s, diff_color_v, diff_density);
        NODELET_INFO("Diff:[%.3lf,%.3lf],[%.3lf, %.3lf, %.3lf], %.3lf",
          width_tolerance_, height_tolerance_, color_h_tol_, color_s_tol_, color_v_tol_, density_tolerance_);
        NODELET_INFO("----------");
      }

      if( diff_width <= width_tolerance_ && 
          diff_height <= height_tolerance_ &&
          diff_color_h <= color_h_tol_ &&
          diff_color_s <= color_s_tol_ &&
          diff_color_v <= color_v_tol_ &&
          diff_density <= density_tolerance_){
        //ROS_INFO("Found %s", it->name.c_str());
        return it->name;
      }
    }
    if(print_info_) NODELET_INFO("==========\n\n");
    return "Unknown";  
  }

  void RectBasedObjectDetection::execute(
    const aricc_vision_msgs::RotatedRectArray::ConstPtr& msg,
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg){
    boost::mutex::scoped_lock lock(mutex_);
    
    //ROS_INFO("EXECUTE_0");
    aricc_vision_msgs::RotatedRectArray rects_msg;
    rects_msg = *msg;

    aricc_vision_msgs::ObjectArray msg_detect_objects;
    msg_detect_objects.objects.clear();

    std::vector<aricc_vision_msgs::RotatedRect>::iterator it;
    for( it = rects_msg.rects.begin(); it != rects_msg.rects.end(); ){
     std::string name = detect(*it); 
     if( name == "Unknown" ){
        it = rects_msg.rects.erase(it);
        continue;
      }

      aricc_vision_msgs::Object msg_object;
      msg_object.position.x = it->center.x;
      msg_object.position.y = it->center.y;
      msg_object.position.z = it->center.z;
      msg_object.orientation.x = 0.0;
      msg_object.orientation.y = 0.0;
      msg_object.orientation.z = it->angle;
      msg_object.name   = name;
      if(msg_object.name == "M20" || msg_object.name == "M30"){
        msg_object.orientation.z = 0.0;
      }
      msg_detect_objects.objects.push_back(msg_object);
      ++it;
    }

    msg_detect_objects.header = rects_msg.header;
    msg_detect_objects.header.stamp = ros::Time::now();
    pub_.publish( msg_detect_objects );
    if(debug_){
      model_.fromCameraInfo(camera_info_msg);
      cv::Mat drawing = cv_bridge::toCvCopy(
        image_msg, sensor_msgs::image_encodings::BGR8)->image;
      pubDebug(drawing, msg_detect_objects, rects_msg);
    }
  }

  void RectBasedObjectDetection::pubDebug( cv::Mat& src, 
    aricc_vision_msgs::ObjectArray msg_objects,
    aricc_vision_msgs::RotatedRectArray msg_rects ){
      cv::Scalar green = cv::Scalar( 0, 255, 0);
      cv::Scalar red = cv::Scalar( 0, 0, 255);
      for(size_t i = 0; i < msg_objects.objects.size(); ++i){
        cv::Point3d p_3d;
        cv::Point2d p_2d;
        cv::Scalar color = green;
        if(msg_objects.objects.at(i).name == "Unknown") color = red;
        p_3d.x = -msg_objects.objects.at(i).position.y;
        p_3d.y = -msg_objects.objects.at(i).position.x;
        p_3d.z = msg_objects.objects.at(i).position.z;
        p_2d = model_.project3dToPixel(p_3d);
        cv::putText(src, msg_objects.objects.at(i).name, 
          p_2d, 1, 1, color, 1, 8 );
        cv::circle( src, p_2d, 3, color, -1, 8, 0 );
        cv::Point2d vertices[4];
        for(size_t j = 0; j < 4; ++j){
          p_3d.y = -msg_rects.rects.at(i).points.at(j).x;
          p_3d.x = -msg_rects.rects.at(i).points.at(j).y;
          p_3d.z = msg_rects.rects.at(i).points.at(j).z;
          vertices[j] =  model_.project3dToPixel(p_3d);
        } 
        for(size_t j = 0; j < 4; ++j){
          cv::line( src, vertices[j], vertices[(j+1)%4], color);
        }
      }
      pub_debug_image_.publish(cv_bridge::CvImage(
                               msg_rects.header,
                               sensor_msgs::image_encodings::BGR8,
                               src).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::RectBasedObjectDetection, nodelet::Nodelet);
