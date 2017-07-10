#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

std::vector<std::string> topics_;
std::vector<ros::Subscriber> subscribers_;
ros::Publisher pub_;
ros::Time time_last_, time_now_;
int current_level_ = 0;
double hold_off_time_;
std::string topic_out_;

void twist_cb(const geometry_msgs::Twist::ConstPtr& msg, int32_t priority) {
  time_now_ = ros::Time::now();
  if ( ( priority >= current_level_ ) | 
       ( (time_now_ - time_last_).toSec() > hold_off_time_) ) {
    //if (current_level_ != priority) ROS_DEBUG("New level: %d", priority);
    geometry_msgs::Twist msg_twist;
    current_level_ = priority;
    time_last_ = ros::Time::now();
    msg_twist.linear.x = msg->linear.x;
    msg_twist.linear.y = msg->linear.y;
    msg_twist.angular.z = msg->angular.z;
    pub_.publish(msg_twist);
    time_last_ = time_now_;
  }
}

int loadParams(ros::NodeHandle n){
  XmlRpc::XmlRpcValue list;
  topics_.clear();
  subscribers_.clear();
  ROS_INFO("----------");
  ROS_INFO("aricc_twist_mux: loading topic list ...");
  if(!n.hasParam("twist_mux_topics")){
    ROS_ERROR("Do not have twist_mux_topics list");
    return 0;
  }
  try {
    n.param("hold_off_time", hold_off_time_, 0.5);   
    n.getParam("twist_mux_topics", list);
    n.param<std::string>("topic_out_name", topic_out_, "cmd_vel_mux");
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_INFO("Loaded %d topics",list.size());
    for(size_t i = 0; i < list.size(); ++i){
      ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      topics_.push_back(static_cast<std::string>(list[i]));
      ROS_INFO("%s",topics_[i].c_str());
    }
  }
  catch (ros::Exception e) {
    ROS_ERROR("Parameter not set: %s", e.what());
    ROS_INFO("----------");
    return 0;
  }
  ROS_INFO("----------");
  return 1;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "aricc_twist_mux");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  if(!loadParams(pnh)) return 0;
  time_last_ = ros::Time::now();
  for(size_t i = 0; i < topics_.size(); ++i ){
    subscribers_.push_back( nh.subscribe<geometry_msgs::Twist>
      (topics_[i], 10, boost::bind(twist_cb, _1, i)));
  }
  pub_ = nh.advertise<geometry_msgs::Twist>(topic_out_, 10);
  ros::spin();
  return 0;
}
