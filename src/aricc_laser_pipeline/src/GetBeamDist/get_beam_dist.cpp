#include <aricc_laser_pipeline/get_beam_dist.h>

namespace aricc_laser_pipeline{

  void GetBeamDist::onInit(){
    ConnectionBasedNodelet::onInit();
    pub_ = advertise<aricc_laser_pipeline::BeamDistArray>(
      *pnh_, "output", 1);
    loadAngleList("angles", angle_list_);
  }
  
  void GetBeamDist::subscribe(){
     sub_ = pnh_->subscribe("input", 1, &GetBeamDist::execute, this);
  }

  void GetBeamDist::unsubscribe(){
    sub_.shutdown();
  }

  bool GetBeamDist::loadAngleList(std::string name,
    std::vector<double>& angle_list){
    try{
      XmlRpc::XmlRpcValue list;
      if( !pnh_->hasParam(name) ){
        NODELET_ERROR("Cannot find %s", name.c_str());
        return false;
      }
      pnh_->getParam( name, list );
      ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      angle_list.clear();
      std::string str = "Loaded ";
      str += name;
      str +=":[";
      for(size_t i = 0; i < list.size(); ++i){
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string value = static_cast<std::string>(list[i]);
        str += value;
        str += " ";
        double ang = atof(value.c_str());
        //angle_list.push_back(aricc_utils::deg2Rad(ang));
        angle_list.push_back(ang);
      }
      str += "]";
      NODELET_INFO("%s", str.c_str());
    }
    catch(ros::Exception e){
      NODELET_ERROR("%s", e.what());
      return false;
    }
    return true;
  }

   bool GetBeamDist::getDist(sensor_msgs::LaserScan reading, 
     double angle, double& dist, unsigned int beam = 3){

    double r_min = static_cast<double>(reading.range_min);
    double r_max = static_cast<double>(reading.range_max);

    if( angle < reading.angle_min || angle > reading.angle_max ) {
      ROS_WARN("angle %lf is out of range [%lf, %lf]", angle, reading.angle_min, reading.angle_max);
      return false;
    }

    //Get distance from the center of laser
    double res = 0.0;
    unsigned int cnt = 0;
    unsigned int index = static_cast<unsigned int>
      ( (angle - reading.angle_min)/reading.angle_increment );

    for(unsigned int i = 0; i < beam; ++i){
      double scan = reading.ranges[i+index];
      if( scan > r_max || 
          scan < r_min ||
          isinf(scan)  || 
          isnan(scan)){
        scan = r_max; 
        //ROS_WARN("distance %.3lf is out of range [%.3lf, %.3lf]", scan, r_min, r_max);
        //continue; 
      }
      res += scan; 
      cnt++;
    }
    if( cnt == 0 ){
      return false;
    }
    dist = static_cast<double>(res/cnt);
    return true; 
  }

  void GetBeamDist::execute( const sensor_msgs::LaserScan::ConstPtr& laser_msg){
    aricc_laser_pipeline::BeamDist msg;
    aricc_laser_pipeline::BeamDistArray msgs;
    msgs.values.clear();

    for(size_t i = 0; i < angle_list_.size(); ++i){
      if( !getDist(*laser_msg,aricc_utils::deg2Rad(angle_list_[i]) ,msg.distance)) {
        continue;
      }
      msg.angle = angle_list_[i];
      msgs.values.push_back(msg);
    }
    msgs.header = laser_msg->header;
    msgs.header.stamp = ros::Time::now(); 
    pub_.publish(msgs);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_laser_pipeline::GetBeamDist, nodelet::Nodelet);

