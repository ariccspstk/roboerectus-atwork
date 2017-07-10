#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <youbot_base_align_dock/BaseAction.h>
#include <youbot_base_local_move/BaseAction.h>
#include <youbot_base_align_dock/ServerConfig.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <aricc_utils/geometry_utils.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <utility>
#include <iostream>
#include <assert.h>

class BaseAction{

public:
  typedef actionlib::SimpleActionClient<youbot_base_local_move::BaseAction> 
    ActionClient;
  typedef youbot_base_align_dock::ServerConfig Config;
  enum StateEnum { ACTIVE = 0, SUCCEEDED, ABORTED };
  enum TaskEnum {kFindMarker = 10, kAdjustBase, kFineAdjustBase, kPlugBase};

protected:
  ros::NodeHandle nh_, pnh_;
  actionlib::SimpleActionServer
    <youbot_base_align_dock::BaseAction> action_;
  std::string action_name_;
  boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
  boost::shared_ptr<ActionClient> client_;
  
  double rate_;
  boost::mutex mutex_;
  std::string marker_topic_;
  TaskEnum task_state_;

  bool is_sent_goal_;
  bool is_sub_;
  bool marker_received_;
  bool voltage_received_;
  geometry_msgs::PoseStamped marker_;
  double timeout_;
  double offset_x_;
  
  youbot_base_align_dock::BaseFeedback feedback_;
  youbot_base_align_dock::BaseResult result_;
 
  ros::Time time_now_, time_start_; 

  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_;
  tf::TransformListener tf_;
  tf::MessageFilter<geometry_msgs::PoseStamped>* tf_filter_;
  std::string base_frame_;
   
public:
  BaseAction(std::string name) :
    pnh_("~"),
    tf_(), 
    marker_received_(false),
    is_sent_goal_(false), 
    is_sub_(false),
    task_state_(kFindMarker),
    offset_x_(0),
    action_(nh_, name, boost::bind(&BaseAction::executeCB, this, _1), false),
    action_name_(name){
    ROS_INFO("----------"); 
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&BaseAction::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pnh_.param<double>("rate", rate_, 5.0);
    pnh_.param<double>("offset_x", offset_x_, 0.3);
    pnh_.param<std::string>("marker_topic", marker_topic_, "detected_marker");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    client_.reset( new ActionClient("youbot_base_local_move_server", true) );
    while(!client_->waitForServer(ros::Duration(5.0))){
      ROS_ERROR("waiting for youbot_base_local_move actionlib");
    }
    tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>
      (sub_, tf_, base_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&BaseAction::markerCB, this, _1) );
    action_.start();
    ROS_INFO("Starting %s ...", name.c_str());
  }

  ~BaseAction(void){
    unsubscribe();
    delete tf_filter_;
  }

  void subscribe(){
    //sub_ = nh_.subscribe( marker_topic_  ,1, &BaseAction::markerCB,this);
    if(!is_sub_) {
      sub_.subscribe( nh_, marker_topic_, 10 );
      is_sub_ = true;
    }
  }

  void unsubscribe(){
    //sub_.shutdown();
    sub_.unsubscribe();
    is_sub_ = false;
  }

  void configCallback(Config &config, uint32_t level){
   // boost::mutex::scoped_lock lock(mutex_);
  }

  void markerCB(const boost::shared_ptr
    <const geometry_msgs::PoseStamped>& msg){
    //ROS_INFO("Reveived Marker");
    boost::mutex::scoped_lock lock(mutex_);
    try{
      //ROS_WARN("%s: msg time: %lf", action_name_.c_str(),msg->header.stamp.toSec());
      //ROS_WARN("%s: Marker time: %lf", action_name_.c_str(),marker_.header.stamp.toSec());
      tf_.transformPose(base_frame_, *msg, marker_);  
    }
    catch (tf::TransformException &ex){
      ROS_ERROR("Failure %s\n", ex.what());
      return;
    }
  }

  StateEnum stopBase(){
    geometry_msgs::Pose2D pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;
    youbot_base_local_move::BaseGoal goal;
    goal.timeout = 60;
    goal.destination = pose;
    client_->sendGoal(goal);
    client_->waitForResult(ros::Duration(0.5));
    if( client_->getState().toString() == "SUCCEEDED" ){
      is_sent_goal_ = false;
      return SUCCEEDED;
    }
    else if( client_->getState().toString() == "ABORTED" ){
      is_sent_goal_ = false;
      return ABORTED;
    }
  }

  StateEnum moveBase(geometry_msgs::Pose2D pose){
    if(!is_sent_goal_){
      youbot_base_local_move::BaseGoal goal;
      goal.timeout = timeout_/2;
      goal.destination = pose;
      client_->sendGoal(goal);
      is_sent_goal_ = true;
      return ACTIVE;
    }
    if( client_->getState().toString() == "SUCCEEDED" ){ 
      is_sent_goal_ = false;   
      return SUCCEEDED;
    }
    else if( client_->getState().toString() == "ABORTED" ){ 
      is_sent_goal_ = false;
      return ABORTED; 
    }
    else return ACTIVE; 
  }

  StateEnum findMarker(){
    return SUCCEEDED;
  }

  StateEnum plugBase(){
    /*
    bool vol_trigger = false;
    {//For mutex
      boost::mutex::scoped_lock lock(mutex_);
      vol_trigger = voltage_received_;
    }
    */
    
    geometry_msgs::Pose2D goal;
    goal.x = -0.05;
    goal.y = 0.0;
    goal.theta = 0.0;
    StateEnum state = moveBase(goal);
    
    //if(!vol_trigger) return ACTIVE;
    //return SUCCEEDED;
    return state;
  }

  bool checkPose(geometry_msgs::PoseStamped& pose){
    boost::mutex::scoped_lock lock(mutex_);
    ros::Time now = ros::Time::now();
    //ROS_WARN("%s: Time now: %lf", action_name_.c_str(),now.toSec());
    //ROS_WARN("%s: Marker time: %lf", action_name_.c_str(),marker_.header.stamp.toSec());
    if( ( now - marker_.header.stamp ).toSec() > 0.5 ){
      ROS_WARN("%s: Marker received, but delayed %lf", action_name_.c_str(),
        ( now - marker_.header.stamp ).toSec() );
      return false;
    }
    pose = marker_;
    return true;
  }

  StateEnum fineAdjustBase(){
    geometry_msgs::PoseStamped pose;
    if(!checkPose(pose)) return ACTIVE;

    geometry_msgs::Pose2D goal;
    goal = getLocalMoveGoal(pose, false, 0.0);
    StateEnum state = moveBase(goal);
    return state;
    //return ACTIVE;
  }

  StateEnum adjustBase(){
    geometry_msgs::PoseStamped pose;
    if(!checkPose(pose)) return ACTIVE;
    geometry_msgs::Pose2D goal;
    goal = getLocalMoveGoal(pose, true, offset_x_);
    StateEnum state = moveBase(goal);
    return state;
    //return ACTIVE;
  }


  geometry_msgs::Pose2D getLocalMoveGoal(geometry_msgs::PoseStamped pose,
    bool move_x, double offset_x = 0.0){
    //Rotate coordinate
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    aricc_utils::quat2Euler(pose.pose.orientation, roll, pitch, yaw); 
    roll = aricc_utils::smallAngle(roll);
    pitch = aricc_utils::smallAngle(pitch);
    yaw = aricc_utils::smallAngle(yaw);
    tf::Vector3 tp(pose.pose.position.x, pose.pose.position.y,
      pose.pose.position.z);
    tf::Matrix3x3 rp;
    rp.setIdentity();
    tf::Pose p(rp, tp);
    ROS_INFO("POS_1:%lf,%lf,%lf",p.getOrigin().getX(), p.getOrigin().getY(), 
      p.getOrigin().getZ());
    
    ROS_INFO("ORT:%lf,%lf,%lf",aricc_utils::rad2Deg(roll), 
      aricc_utils::rad2Deg(pitch), aricc_utils::rad2Deg(yaw));

    tf::Vector3 v1(0.0, 0.0, 0.0);
    tf::Matrix3x3 m1;
    m1.setRPY(0.0, 0.0, -yaw);
    tf::Transform rot_in_robot_frame(m1, v1);
    
    p = rot_in_robot_frame * p;
    //ROS_INFO("POS_2:%lf,%lf,%lf",p.getOrigin().getX(), p.getOrigin().getY(), 
    //  p.getOrigin().getZ());

    double x = p.getOrigin().getX();
    double y = p.getOrigin().getY();
    double z = p.getOrigin().getZ();
    if(move_x) x += offset_x;
    else x = 0.0;
    
    tf::Vector3 v3;
    v3.setValue(x,y,z);
    p.setOrigin(v3);

    p = rot_in_robot_frame.inverse() * p;
    //ROS_INFO("POS_3:%lf,%lf,%lf",p.getOrigin().getX(), p.getOrigin().getY(), 
    //  p.getOrigin().getZ());

    geometry_msgs::Pose2D goal;
    goal.x = p.getOrigin().getX();
    goal.y = p.getOrigin().getY();
    goal.theta = yaw;
    return goal;
  }

  StateEnum taskStep(){
    StateEnum state;
    //ROS_INFO("Current task:%d",task_state_);
    switch(task_state_){
      case kFindMarker:
        state = findMarker();
        if(state == SUCCEEDED){ 
          task_state_ = kAdjustBase;
        }
        else if(state == ABORTED) return ABORTED;
      break;
      
      case kAdjustBase:
        state = adjustBase();
        if(state == SUCCEEDED) task_state_ = kFineAdjustBase;
        //if(state == SUCCEEDED) return SUCCEEDED;
      break;

      case kFineAdjustBase:
        state = fineAdjustBase();
        if(state == SUCCEEDED) task_state_ = kPlugBase;
      break;

      case kPlugBase:
        state = plugBase();
        if(state == SUCCEEDED) return SUCCEEDED;
        else if(state == ABORTED) return ABORTED;
      break;
    }
    return ACTIVE;
  }

  void resetState(){
    stopBase();
    is_sent_goal_ = false;
    time_start_ = ros::Time::now();
    task_state_ = kFindMarker;
    client_->cancelAllGoals();
    unsubscribe();
  }

  int isGoalValid( youbot_base_align_dock::BaseGoal goal){
    ROS_INFO("%s: accept goal", action_name_.c_str());
    if( isnan(goal.timeout) || isinf(goal.timeout) ){
      ROS_WARN("Goal has NAN item");
      return 0;
    }
    ROS_INFO("Goal: %.3lf",goal.timeout);
    timeout_ = goal.timeout;
    return 1;
  }

  void setAborted(youbot_base_align_dock::BaseResult result, 
    std::string str = ""){
    ROS_ERROR("%s: Aborted, %s", action_name_.c_str(), str.c_str());
    action_.setAborted(result);
    resetState();
  }

  void setSucceeded(youbot_base_align_dock::BaseResult result){
    ROS_INFO("%s: Succeeded, take %.3lf s", action_name_.c_str(),
      (time_now_ - time_start_).toSec());
    action_.setSucceeded(result);
    resetState();
  }

  void setPreempted(){
    ROS_WARN("%s: Preempted", action_name_.c_str());
    action_.setPreempted();
    resetState();
  }

  void executeCB(const youbot_base_align_dock::BaseGoalConstPtr &goal_msg){
    youbot_base_align_dock::BaseGoal goal = *goal_msg;
    resetState();

    if( isGoalValid(goal) == 0 ){
      setAborted(result_,"Goal is invalid");
      return;
    }
    ros::Rate r(rate_);
    
    while(action_.isActive() || ros::ok()){
      subscribe();
      time_now_ = ros::Time::now(); 
      if( (time_now_ - time_start_).toSec() >= goal.timeout && 
        goal.timeout != 0.0 ){
        setAborted(result_, "Timeout");
        return;
      }

      if( action_.isPreemptRequested() ){
        ROS_INFO("Preempt Requested");
        if( action_.isNewGoalAvailable() ){
          goal = *(action_.acceptNewGoal());
          if( isGoalValid(goal) == 0 ){
            action_.setAborted(result_, "Goal is invalid");
            return;
          }
          else {
            ROS_INFO("%s: accept new goal", action_name_.c_str());
            resetState();
          }
        }
        else{ setPreempted(); return; }
      }
      //control loop
      StateEnum state = taskStep();
      if( state == SUCCEEDED ) { setSucceeded(result_); return; }
      else if( state == ABORTED ) { setAborted(result_); return; }
      r.sleep();
      if( r.cycleTime() > ros::Duration(1.0/rate_) )
        ROS_WARN("%s: Control desired rate of %.3lfHz... the loop actually took %.4lf seconds", action_name_.c_str(), rate_, r.cycleTime().toSec());
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "youbot_base_align_dock_server");
  BaseAction action(ros::this_node::getName());
  ros::spin();
  return 0;
}
