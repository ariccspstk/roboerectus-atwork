#include <actionlib/client/simple_action_client.h>
#include <youbot_arm_joints/JointAction.h>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <assert.h>

class YoubotTeleop{
public:
  typedef actionlib::SimpleActionClient<youbot_arm_joints::JointAction> ActionClient;

  YoubotTeleop():nh_("~"),rate_(5),btnCounterArm_(0),btnNumArm_(0),
    btnCounterGripper_(0),btnNumGripper_(0),arm_power_on_(true),
    base_power_on_(true){
  }

  ~YoubotTeleop(){
    pub_vel_.shutdown();
    sub_joy_.shutdown();
  }
  
  bool init(){
    loadParams();
    if(!loadAllJoints()) return false;
    if(!loadInitPose("init_pose",init_pose_)) return false;

    client_.reset( new ActionClient("youbot_arm_joints_server", true) );
    unsigned int cnt = 0;
    while(!client_->waitForServer(ros::Duration(5.0))){
      ROS_ERROR("youbot_arm_joints_server is not excuting");
      if(++cnt == 5) return false;
    }
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    pub_init_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);
    sub_joy_ = nh_.subscribe("/joy", 1, &YoubotTeleop::joyCallback, this); 
    client_base_power_on_ = nh_.serviceClient<std_srvs::Empty>
      ("/base/switchOnMotors");
    client_base_power_off_ = nh_.serviceClient<std_srvs::Empty>
      ("/base/switchOffMotors");
    client_arm_power_on_ = nh_.serviceClient<std_srvs::Empty>
      ("/arm_1/switchOnMotors");
    client_arm_power_off_ = nh_.serviceClient<std_srvs::Empty>
      ("/arm_1/switchOffMotors");
    return true;     
  }
  
  void spin(){
    ros::Rate loop_rate(rate_);
    while( nh_.ok() ){
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_vel_;
  ros::Publisher pub_init_pose_;
  ros::ServiceClient client_base_power_on_;
  ros::ServiceClient client_base_power_off_;
  ros::ServiceClient client_arm_power_on_;
  ros::ServiceClient client_arm_power_off_;
  
  ros::Subscriber sub_joy_;
  geometry_msgs::PoseWithCovariance init_pose_;

  std::vector< std::vector<double> > arm_joints_list_;
  std::vector< std::vector<double> > gripper_joints_list_;
  std::vector<std::string> arm_name_list_;
  std::vector<std::string> gripper_name_list_;

  unsigned int btnCounterArm_;
  unsigned int btnNumArm_;
  unsigned int btnCounterGripper_;
  unsigned int btnNumGripper_;

  bool arm_power_on_;
  bool base_power_on_;

  unsigned int rate_;
  int axis_linear_x_;
  int axis_linear_y_;
  int axis_angular_;
  double scale_linear_;
  double scale_angular_;
  boost::shared_ptr<ActionClient> client_;
  
  void moveArm(std::string name, std::vector<double> joints){
    youbot_arm_joints::JointGoal goal;
    goal.timeout = 10.0;
    goal.name = name;
    goal.joints = joints;
    client_->sendGoal(goal);
  }

  void loadParams(){
    nh_.param<int>("axis_linear_x",   axis_linear_x_, 1);
    nh_.param<int>("axis_linear_y",    axis_linear_y_, 0);
    nh_.param<int>("axis_angular",     axis_angular_, 2);
    nh_.param<double>("scale_linear",  scale_linear_, 0.1);
    nh_.param<double>("scale_angular", scale_angular_, 0.2);
  }

  bool loadNameList(std::string name, 
    std::vector<std::string>& name_list){
    try{
      XmlRpc::XmlRpcValue list;
      if( !nh_.hasParam(name) ){
        ROS_ERROR("Cannot find %s", name.c_str());
        return false;
      }
      nh_.getParam( name, list );
      ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      name_list.clear();
      std::string str = "Loaded ";
      str += name;
      str +=":[";
      for(size_t i = 0; i < list.size(); ++i){
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string value = static_cast<std::string>(list[i]);
        str += value;
        str += " ";
        name_list.push_back( value );
      }
      str += "]";
      ROS_INFO("%s", str.c_str());
    }
    catch(ros::Exception e){
      ROS_ERROR("%s", e.what());
      return false;
    }
    return true;
  }

  bool loadJoints(std::string name, std::vector<double>& joints){
    //ROS_INFO("Loading arm joints ...");
    try{
      XmlRpc::XmlRpcValue list;
      if( !nh_.hasParam(name) ){
        ROS_ERROR("Cannot find %s", name.c_str());
        return false;
      }
      nh_.getParam( name, list );
      ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      //ROS_INFO("Found list size:%d", list.size());
      joints.clear();
      std::string str = "Loaded ";
      str += name;
      str +=":[";
      for(size_t i = 0; i < list.size(); ++i){
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string value = static_cast<std::string>(list[i]);
        str += value;
        str += " ";
        joints.push_back( atof(value.c_str()) );
      }
      str += "]";
      ROS_INFO("%s", str.c_str());
    }
    catch(ros::Exception e){
      ROS_ERROR("%s", e.what());
      return false;
    }
    return true;
  }

  bool loadAllJoints(){
    if(!loadNameList("arm_name_list",     arm_name_list_)) return false;
    if(!loadNameList("gripper_name_list", gripper_name_list_)) return false;
    arm_joints_list_.clear();
    gripper_joints_list_.clear();
    std::vector<double> joints;
    for(size_t i = 0; i < arm_name_list_.size(); ++i){
      joints.clear();
      if(!loadJoints(arm_name_list_[i],joints))
        return false;
      arm_joints_list_.push_back(joints);
    }
    for(size_t i = 0; i < gripper_name_list_.size(); ++i){
      joints.clear();
      if(!loadJoints(gripper_name_list_[i],joints))
        return false;
      gripper_joints_list_.push_back(joints);
    }
    btnNumArm_ = arm_joints_list_.size();
    btnNumGripper_ = gripper_joints_list_.size();
    return true;
  }

  bool loadInitPose(std::string name, 
    geometry_msgs::PoseWithCovariance& pose){
    try{
      XmlRpc::XmlRpcValue list;
      if( !nh_.hasParam(name) ){
        ROS_ERROR("Cannot find %s", name.c_str());
        return false;
      }
      nh_.getParam( name, list );
      ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      if(list.size() != 7){
        ROS_ERROR("%s size should be 7, but it is %d", 
          name.c_str(), list.size());
        return false;
      }
      
      std::string value;
      ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeString);
      value = static_cast<std::string>(list[0]);
      pose.pose.position.x = atof(value.c_str());
      ROS_ASSERT(list[1].getType() == XmlRpc::XmlRpcValue::TypeString);
      value = static_cast<std::string>(list[1]);
      pose.pose.position.y = atof(value.c_str());
      ROS_ASSERT(list[2].getType() == XmlRpc::XmlRpcValue::TypeString);
      value = static_cast<std::string>(list[2]);
      pose.pose.position.z = atof(value.c_str());

      ROS_ASSERT(list[3].getType() == XmlRpc::XmlRpcValue::TypeString);
      value = static_cast<std::string>(list[3]);
      pose.pose.orientation.x = atof(value.c_str());
      ROS_ASSERT(list[4].getType() == XmlRpc::XmlRpcValue::TypeString);
      value = static_cast<std::string>(list[4]);
      pose.pose.orientation.y = atof(value.c_str());
      ROS_ASSERT(list[5].getType() == XmlRpc::XmlRpcValue::TypeString);
      value = static_cast<std::string>(list[5]);
      pose.pose.orientation.z = atof(value.c_str());
      ROS_ASSERT(list[6].getType() == XmlRpc::XmlRpcValue::TypeString);
      value = static_cast<std::string>(list[6]);
      pose.pose.orientation.w = atof(value.c_str());
    }
    catch(ros::Exception e){
      ROS_ERROR("%s", e.what());
      return false;
    }
    ROS_INFO("loaded init pose: [%.3lf, %.3lf, %.3lf], [%.3lf, %.3lf, %.3lf, %.3lf]", pose.pose.position.x,pose.pose.position.y,pose.pose.position.z, pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
    return true;
  
  }

  void setInitPose(){
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/map";
    msg.pose = init_pose_;
    pub_init_pose_.publish(msg);
  }

  void moveBase(double axis_x, double axis_y, double axis_angular){
    geometry_msgs::Twist msg_vel;
    msg_vel.linear.x  = axis_x * scale_linear_;
    msg_vel.linear.y  = axis_y * scale_linear_;
    msg_vel.angular.z = axis_angular * scale_angular_;
    pub_vel_.publish( msg_vel );
  }

  void switchBasePower(){
    std_srvs::Empty srv;
    if(base_power_on_){
      //ROS_INFO("Switch off base");
      if(client_base_power_off_.call(srv)) base_power_on_ = false;
    }
    else{
      //ROS_INFO("Switch on base");
      if(client_base_power_on_.call(srv)) base_power_on_ = true;
    } 
  }

  void switchArmPower(){
    std_srvs::Empty srv;
    if(arm_power_on_){
      //ROS_INFO("Switch off arm");
      if(client_arm_power_off_.call(srv)) arm_power_on_ = false;
    }
    else{
      //ROS_INFO("Switch on arm");
      if(client_arm_power_on_.call(srv)) arm_power_on_ = true;
    } 
  }

  void joyCallback(const sensor_msgs::JoyConstPtr& msg){
    ros::Time now = ros::Time::now();
    if( ( now - msg->header.stamp ).toSec() > 0.5 ){
      ROS_WARN("youbot_teleop: Joy msg received, but delayed %lf",
        ( now - msg->header.stamp ).toSec() );
      return;
    }

    if( msg->axes.size() < 3){
      ROS_ERROR( "Too few joystick axes: %ld (expected more than 3)", msg->axes.size() );
      return;
    }
    if(msg->buttons[4] == 1){
      switchBasePower();
    }
    if(msg->buttons[5] == 1){
      switchArmPower();
    }
    if(msg->buttons[2] == 1){
      setInitPose();
    }
        
    if(arm_power_on_){
      if(msg->buttons[0] == 1){
        moveArm( "arm", arm_joints_list_.at( (btnCounterArm_++)%btnNumArm_) );
        if(btnCounterArm_ == btnNumArm_) btnCounterArm_ = 0;
      }
      if(msg->buttons[1] == 1){
        moveArm( "gripper", gripper_joints_list_.at
          ( (btnCounterGripper_++)%btnNumGripper_) );
        if(btnCounterGripper_ == btnNumGripper_) btnCounterGripper_ = 0;
      }
    }

    if(base_power_on_) moveBase(msg->axes[axis_linear_x_],
      msg->axes[axis_linear_y_],msg->axes[axis_angular_]);
  }

};

int main( int argc, char** argv ){
  ros::init( argc, argv, "youbot_teleop" );
  YoubotTeleop rt;
  if(!rt.init()){
    ROS_ERROR("youbot teleop initialization error");
    return 0;
  }
  rt.spin();
  return 0;
}

