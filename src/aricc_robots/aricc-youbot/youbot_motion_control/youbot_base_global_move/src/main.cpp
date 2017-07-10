#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <utility>
#include <assert.h>

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <youbot_base_global_move/BaseAction.h>
#include <youbot_base_local_move/BaseAction.h>
#include <move_base_msgs/MoveBaseAction.h>

class BaseAction{

public:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ActionGlobal;
  typedef actionlib::SimpleActionClient<youbot_base_local_move::BaseAction> ActionLocal;
  enum StateEnum { ACTIVE = 0, SUCCEEDED, ABORTED };

protected:
  ros::NodeHandle nh_, pnh_;
  actionlib::SimpleActionServer<youbot_base_global_move::BaseAction> action_;
  std::string action_name_;
  bool is_base_local_sent_goal_;
  bool is_base_global_sent_goal_;
  double rate_;
  boost::mutex mutex_;

  youbot_base_global_move::BaseFeedback feedback_;
  youbot_base_global_move::BaseResult result_;
 
  ros::Publisher pub_arm_;
  ros::Publisher pub_gripper_;
  ros::Subscriber sub_;
  ros::Time time_now_, time_start_;
  boost::shared_ptr<ActionGlobal> client_global_;
  boost::shared_ptr<ActionLocal>  client_local_;
 
public:
  BaseAction(std::string name) :
    pnh_("~"),
    is_base_local_sent_goal_(false),
    is_base_global_sent_goal_(false),
    action_(nh_, name, boost::bind(&BaseAction::executeCB, this, _1), false),
    action_name_(name){
    ROS_INFO("----------"); 
    pnh_.param<double>("rate", rate_, 5);
    client_global_.reset( new ActionGlobal("move_base", true) );
    client_local_.reset( new ActionLocal("youbot_base_local_move_server", true) );
    while(!client_global_->waitForServer(ros::Duration(5.0))){
        ROS_ERROR("Waiting for Move_base actionlib");
    }
    while(!client_local_->waitForServer(ros::Duration(5.0))){
        ROS_ERROR("Waiting for youbot_base_local_move actionlib");
    }
    action_.start();
    ROS_INFO("Starting %s ...", name.c_str());
  }

  ~BaseAction(void){
  }

  StateEnum moveBase(geometry_msgs::Pose pose){
    if(!is_base_global_sent_goal_){
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = pose;
      client_global_->sendGoal(goal);
      is_base_global_sent_goal_ = true;
      return ACTIVE;
    }
    if( client_global_->getState().toString() == "SUCCEEDED" ){ 
      is_base_global_sent_goal_ = false;
      return SUCCEEDED;
    }
    else if( client_global_->getState().toString() == "ABORTED" ){ 
      is_base_global_sent_goal_ = false;
      return ABORTED;
    }
    else return ACTIVE;
  }
  
  StateEnum stopBase(){
    geometry_msgs::Pose2D pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;
    youbot_base_local_move::BaseGoal goal;
    goal.timeout = 60;
    goal.destination = pose;
    client_local_->sendGoal(goal);
    client_local_->waitForResult(ros::Duration(0.5));
    if( client_local_->getState().toString() == "SUCCEEDED" ){
      is_base_local_sent_goal_ = false;
      return SUCCEEDED;
    }
    else if( client_local_->getState().toString() == "ABORTED" ){
      is_base_local_sent_goal_ = false;
      return ABORTED;
    }
  }
  
  void resetState(){
    stopBase();
    is_base_local_sent_goal_ = false;
    is_base_global_sent_goal_ = false;
    time_start_ = ros::Time::now();
    client_global_->cancelGoal();
    client_local_->cancelGoal();
  }
  
  int isGoalValid( youbot_base_global_move::BaseGoal goal){
    ROS_INFO("%s: accept goal", action_name_.c_str());
    if( isnan(goal.destination.position.x) ||
        isnan(goal.destination.position.y) ||
        isnan(goal.destination.position.z) ||
        isnan(goal.destination.orientation.x) ||
        isnan(goal.destination.orientation.y) ||
        isnan(goal.destination.orientation.z) ||
        isnan(goal.destination.orientation.w) ) return 0;
    if( isinf(goal.destination.position.x) ||
        isinf(goal.destination.position.y) ||
        isinf(goal.destination.position.z) ||
        isinf(goal.destination.orientation.x) ||
        isinf(goal.destination.orientation.y) ||
        isinf(goal.destination.orientation.z) ||
        isinf(goal.destination.orientation.w) ) return 0;
    
    ROS_INFO("Goal: %.3lf, (%.3lf,%.3lf,%.3lf), (%.3lf,%.3lf,%.3lf,%.3lf)",goal.timeout, 
        goal.destination.position.x, goal.destination.position.y, 
        goal.destination.position.z, goal.destination.orientation.x,
        goal.destination.orientation.y, goal.destination.orientation.z,
        goal.destination.orientation.w);
    return 1;
  }

  void setAborted(youbot_base_global_move::BaseResult result, std::string str = ""){
    ROS_ERROR("%s: Aborted, %s", action_name_.c_str(), str.c_str());
    action_.setAborted(result);
    resetState();
  }

  void setSucceeded(youbot_base_global_move::BaseResult result){
    ROS_INFO("%s: Succeeded, take %lf s", action_name_.c_str(),
      (time_now_ - time_start_).toSec());
    action_.setSucceeded(result);
    resetState();
  }

  void setPreempted(){
    ROS_WARN("%s: Preempted", action_name_.c_str());
    action_.setPreempted();
    resetState();
  }

  void executeCB(const youbot_base_global_move::BaseGoalConstPtr &goal_msg){
    youbot_base_global_move::BaseGoal goal = *goal_msg;
    if( isGoalValid(goal) == 0 ){
      setAborted(result_,"Goal is invalid");
      return;
    }

    resetState();
    time_start_ = ros::Time::now();
    ros::Rate r(rate_);
    while(action_.isActive() || ros::ok()){
      time_now_ = ros::Time::now(); 
      if((time_now_ - time_start_).toSec() >= goal.timeout &&
        goal.timeout != 0.0){
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
          }
        }
        else{ setPreempted(); return; }
      }

      //control loop
      StateEnum state = moveBase(goal.destination);

      if( state == SUCCEEDED ){ setSucceeded(result_); return; }
      //else if( state == ABORTED )  { setAborted(result_); return; }
      else if( state == ABORTED ){ 
        ROS_ERROR("%s: goal is aborted, try it again", action_name_.c_str());
        resetState(); 
      }

      r.sleep();
      if( r.cycleTime() > ros::Duration(1.0/rate_) )
        ROS_WARN("%s: Control desired rate of %.3lfHz... the loop actually took %.4lf seconds", action_name_.c_str(), rate_, r.cycleTime().toSec());
    }
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "youbot_base_global_move_server");
  BaseAction action(ros::this_node::getName());
  ros::spin();
  return 0;
}
