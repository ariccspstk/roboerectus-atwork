#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <youbot_base_alignment/BaseAction.h>
#include <youbot_base_local_move/BaseAction.h>
#include <youbot_base_alignment/ServerConfig.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <aricc_laser_pipeline/BeamDistArray.h>
#include <tf/transform_broadcaster.h>
#include <aricc_utils/geometry_utils.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <utility>
#include <iostream>
#include <assert.h>

#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_RESET   "\x1b[0m"

class BaseAction{

public:
  typedef actionlib::SimpleActionClient<youbot_base_local_move::BaseAction> 
    ActionClient;
  typedef youbot_base_alignment::ServerConfig Config;
  enum StateEnum { ACTIVE = 0, SUCCEEDED, ABORTED };
  enum TaskEnum {kAdjustAngle = 10, kAdjustDist, kAdjustAngleAgain};

protected:
  ros::NodeHandle nh_, pnh_;
  actionlib::SimpleActionServer
    <youbot_base_alignment::BaseAction> action_;
  std::string action_name_;
  boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
  boost::shared_ptr<ActionClient> client_;
  
  double rate_;
  double offset_y_;
  double limit_y_;
  double l_dist_, r_dist_, f_dist_, right_dist_;
  double l_angle_, r_angle_, f_angle_, right_angle_;
  double width_;
  double align_dist_;
  double timeout_;
  
  boost::mutex mutex_;
  std::string topic_name_;
  TaskEnum task_state_;

  bool is_sent_goal_;
  aricc_laser_pipeline::BeamDistArray beam_msg_;
  
  youbot_base_alignment::BaseFeedback feedback_;
  youbot_base_alignment::BaseResult result_;
 
  ros::Subscriber sub_;
  ros::Time time_now_, time_start_; 
 
public:
  BaseAction(std::string name) :
    pnh_("~"), is_sent_goal_(false),
    task_state_(kAdjustAngle),
    align_dist_(0),
    offset_y_(0),
    action_(nh_, name, boost::bind(&BaseAction::executeCB, this, _1), false),
    action_name_(name){
    ROS_INFO("----------"); 
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&BaseAction::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pnh_.param<double>("rate", rate_, 5.0);
    pnh_.param<double>("offset_y", offset_y_, 0.15);
    pnh_.param<double>("robot_width",  width_, 0.54);

    pnh_.param<double>("l_angle", l_angle_, 15);
    pnh_.param<double>("f_angle", f_angle_, 0.0);
    pnh_.param<double>("r_angle", r_angle_, -15);
    pnh_.param<double>("right_angle", right_angle_, 0);
    pnh_.param<std::string>("topic_name", topic_name_, "base_scan");
    client_.reset( new ActionClient("youbot_base_local_move_server", true) );
    while(!client_->waitForServer(ros::Duration(5.0))){
      ROS_ERROR("waiting for youbot_base_local_move actionlib");
    }
    action_.start();
    ROS_INFO("Starting %s ...", name.c_str());
  }

  ~BaseAction(void){
    sub_.shutdown();
  }

  void subscribe(){
    sub_ = nh_.subscribe( topic_name_  ,1, &BaseAction::beamDistCB,this);
    ROS_INFO("Start Subscribing:%s",topic_name_.c_str());
  }

  void unsubscribe(){
    sub_.shutdown();
  }

  void configCallback(Config &config, uint32_t level){
  }

  void beamDistCB(const aricc_laser_pipeline::BeamDistArray::ConstPtr& msg){
    boost::mutex::scoped_lock lock(mutex_);
    beam_msg_ = *msg;
    //double time_diff =
    //  (ros::Time::now() - beam_msg_.header.stamp).toSec();
    //ROS_INFO("Beam msg: Data is older than %.3lf",  time_diff);
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

  double calAdjustAngle(){
    double l_angle = aricc_utils::deg2Rad(l_angle_);
    double r_angle = aricc_utils::deg2Rad(r_angle_);

    double diff = fabs(l_dist_ - r_dist_);
    double dist_1 = fabs(l_dist_*sin(l_angle)) + 
      fabs(r_dist_*sin(r_angle));
    double dist_2 = fabs(l_dist_*cos(l_angle)) - 
      fabs(r_dist_*cos(r_angle));
    double angle = atan(dist_2/dist_1);
    //ROS_INFO("Dist:(%.3lf,%.3lf)",l_dist_,r_dist_);
    //ROS_INFO("Angle:(%.3lf,%.3lf)",angle, aricc_utils::rad2Deg(angle));
    return angle;
  }

  double calAdjustDist(){
    double dist = fabs( f_dist_ - align_dist_ );
    return dist;
  }

  StateEnum adjustAngle(){
    boost::mutex::scoped_lock lock(mutex_);
    if(beam_msg_.values.empty()) return ACTIVE;
    double time_diff =
      (ros::Time::now() - beam_msg_.header.stamp).toSec();
    if( time_diff > 0.5 ){
      //ROS_WARN("Beam msg: Data is older than %.3lf",  time_diff);
      return ACTIVE;
    }
    unsigned int result = 0; 
    for(  std::vector<aricc_laser_pipeline::BeamDist>::iterator it = beam_msg_.values.begin(); it !=  beam_msg_.values.end(); ++it){
      if(it->angle == l_angle_){ 
        l_dist_ = it->distance;
        result++;
      }
      if(it->angle == f_angle_){ 
        f_dist_ = it->distance;
        result++;
      }
      if(it->angle == r_angle_){ 
        r_dist_ = it->distance;
        result++;
      }
      if(it->angle == right_angle_){ 
        right_dist_ = it->distance;
        result++;
      }
    }
    if(result != 4) return ACTIVE;

    geometry_msgs::Pose2D goal;
    double angle = calAdjustAngle();
    //ROS_INFO("Angle:%.3lf", angle);
    if(isnan(angle) || isinf(angle)) return ABORTED; 
    goal.x = 0.0;
    goal.y = 0.0;
    goal.theta = -angle;
    StateEnum state = moveBase(goal);
    //ROS_INFO("Adjusting angle, state:%d", state);
    return state;
    //return ACTIVE; 
  }

  StateEnum adjustDist(){
    boost::mutex::scoped_lock lock(mutex_);
    if(beam_msg_.values.empty()) return ACTIVE;
    double time_diff =
      (ros::Time::now() - beam_msg_.header.stamp).toSec();
    if( time_diff > 0.5 ){
      //ROS_WARN("Beam msg: Data is older than %.3lf",  time_diff);
      return ACTIVE;
    }
    unsigned int result = 0;     
    for(  std::vector<aricc_laser_pipeline::BeamDist>::iterator it = beam_msg_.values.begin(); it !=  beam_msg_.values.end(); ++it){
      if(it->angle == l_angle_){
        l_dist_ = it->distance;
        result++;
      }
      if(it->angle == f_angle_){
        f_dist_ = it->distance;
        result++;
      }
      if(it->angle == r_angle_){
        r_dist_ = it->distance;
        result++;
      }
      if(it->angle == right_angle_){
        right_dist_ = it->distance;
        result++;
      }
    }
    if(result != 4) return ACTIVE; 

    geometry_msgs::Pose2D goal;
    double dist = calAdjustDist();
    if(isnan(dist) || isinf(dist)) return ABORTED; 
    goal.x = dist;
    
    //Calculate y distance
    double limit_dist = -( right_dist_ - (width_/2.0) );
    if( offset_y_ <= limit_dist ) goal.y = limit_dist;
    else goal.y = offset_y_;
    goal.theta = 0.0;
    StateEnum state = moveBase(goal);
    //ROS_INFO("dist:%.3lf, %.3lf", right_dist_, limit_dist);
    return state;
    //return ACTIVE;
  }

  StateEnum taskStep(){
    StateEnum state;
    switch(task_state_){
      case kAdjustAngle:
        state = adjustAngle();
        if(state == SUCCEEDED){ 
          task_state_ = kAdjustDist;
          sleep(1);
        }
        else if(state == ABORTED) return ABORTED;
      break;
      
      case kAdjustDist:
        state = adjustDist();
        if(state == SUCCEEDED) return SUCCEEDED;
        //if(state == SUCCEEDED) task_state_ = kAdjustAngleAgain;
        else if(state == ABORTED) return ABORTED;
      break;

      case kAdjustAngleAgain:
        state = adjustAngle();
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
    task_state_ = kAdjustAngle;
    client_->cancelAllGoals();
    unsubscribe();
  }

  int isGoalValid( youbot_base_alignment::BaseGoal goal){
    ROS_INFO("%s: accept goal", action_name_.c_str());
    if( isnan(goal.timeout) || isinf(goal.timeout) ||
        isnan(goal.distance) || isinf(goal.distance)){
      ROS_WARN("Goal has NAN item");
      return 0;
    }
    ROS_INFO("Goal: %.3lf, %.3lf",goal.timeout, goal.distance);
    timeout_ = goal.timeout;
    align_dist_ = goal.distance;
    return 1;
  }

  void setAborted(youbot_base_alignment::BaseResult result, 
    std::string str = ""){
    ROS_ERROR("%s: Aborted, %s", action_name_.c_str(), str.c_str());
    action_.setAborted(result);
    resetState();
  }

  void setSucceeded(youbot_base_alignment::BaseResult result){
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

  void executeCB(const youbot_base_alignment::BaseGoalConstPtr &goal_msg){
    youbot_base_alignment::BaseGoal goal = *goal_msg;
    resetState();

    if( isGoalValid(goal) == 0 ){
      setAborted(result_,"Goal is invalid");
      return;
    }
    subscribe();
    ros::Rate r(rate_);
    
    while(action_.isActive() || ros::ok()){ 
      time_now_ = ros::Time::now(); 
      if( (time_now_ - time_start_).toSec() >= goal.timeout && 
        goal.timeout != 0.0 ){
        setAborted(result_, "Timeout");
        return;
      }

      if( action_.isPreemptRequested() ){
        if( action_.isNewGoalAvailable() ){
          goal = *(action_.acceptNewGoal());
          if( isGoalValid(goal) == 0 ){
            action_.setAborted(result_, "Goal is invalid");
            return;
          }
          else {
            ROS_INFO("%s: accept new goal", action_name_.c_str());
            resetState();
            subscribe();
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
  ros::init(argc, argv, "youbot_base_alignment_server");
  BaseAction action(ros::this_node::getName());
  ros::spin();
  return 0;
}
