#include "YoubotGraspIK.h"
#include <iostream>
#include <assert.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <youbot_arm_joints_trajectory/TrajectoryAction.h>
#include <youbot_arm_joints/JointAction.h>
#include <youbot_arm_ik/IkAction.h>
#include <aricc_utils/geometry_utils.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

//YoubotGraspIK IKSolver;

class IkAction{
public:
  //typedef actionlib::SimpleActionClient<youbot_arm_joints::JointAction> ActionClient;
   typedef actionlib::SimpleActionClient<youbot_arm_joints_trajectory::TrajectoryAction> ActionClient;
  typedef joint_positions_solution_t solution_t;
  enum StateEnum { ACTIVE = 0, SUCCEEDED, ABORTED };

protected:
  ros::NodeHandle nh_, pnh_;
  //actionlib stuffs
  actionlib::SimpleActionServer<youbot_arm_ik::IkAction> action_;
  std::string action_name_;
  bool is_sent_goal_;
  double rate_;
  boost::mutex mutex_;
  youbot_arm_ik::IkFeedback feedback_;
  youbot_arm_ik::IkResult   result_;
  ros::Time time_now_, time_start_;
  std::vector<double> joints_state_;
  std::vector<double> solution_;
  ros::Subscriber sub_;
  boost::shared_ptr<ActionClient> client_;

public:
  IkAction(std::string name) :
    pnh_("~"), is_sent_goal_(false),
    action_(nh_, name, boost::bind(&IkAction::executeCB, this, _1), false),
    action_name_(name){
    ROS_INFO("----------");
    ROS_INFO("Starting %s ...", name.c_str());
    pnh_.param<double>("rate", rate_, 20);
    client_.reset( new ActionClient("youbot_arm_joints_trajectory_server", true) );
    while(!client_->waitForServer(ros::Duration(5.0))){
      ROS_ERROR("Waiting for youbot_arm_joints_trajectory_server actionlib");
    }
    action_.start();
    ROS_INFO("Started %s ...", name.c_str());
  }

  ~IkAction(void){
  }

  void subscribe(){
    sub_ = nh_.subscribe("/joint_states" ,1,&IkAction::jointsCB,this);
  }
  
  void unsubscribe(){
    sub_.shutdown();
  }

  void jointsCB(const sensor_msgs::JointState::ConstPtr& msg){
    boost::mutex::scoped_lock lock(mutex_);
    joints_state_.clear();
    if(msg->position.size() != 7 && msg->name[0] == "arm_joint_1"){
      ROS_WARN("It should be 7 joints, received %lu joints!",
        msg->position.size());
      return;
    }
    for(size_t i = 0; i < msg->position.size(); ++i)
      joints_state_.push_back(msg->position[i]);
  }

  
  StateEnum moveArm( std::vector<double> joints, double duration){
    if(!is_sent_goal_){
      ROS_INFO("%s: found solution: [%lf, %lf, %lf, %lf, %lf]", 
        action_name_.c_str(), joints[0], joints[1], joints[2], joints[3],
        joints[4]);
      youbot_arm_joints_trajectory::TrajectoryGoal goal;
      trajectory_msgs::JointTrajectoryPoint segment;
      segment.positions.resize(5);
      for(unsigned int i = 0; i < 5; ++i)
        segment.positions[i] = joints[i];
      segment.time_from_start = ros::Duration(duration);
      goal.timeout = 30.0;
      goal.points.push_back(segment);
      //ROS_INFO("The segments: [%lf, %lf, %lf, %lf, %lf]", segment.positions[0], segment.positions[1], segment.positions[2], segment.positions[3], segment.positions[4]);
      client_->sendGoal(goal);
      is_sent_goal_ = true;
      return ACTIVE;
      //ROS_INFO("Waitting for result ...");
    }
    if( client_->getState().toString() == "SUCCEEDED" ){   
       is_sent_goal_ = false;
       return SUCCEEDED;
    }
    else if( client_->getState().toString() == "ABORTED"){
       is_sent_goal_ = false;
       return ABORTED;
    }
    else return ACTIVE;
  }

  /*
  StateEnum moveArm(std::vector<double> joints){
    if(!is_sent_goal_){
      ROS_INFO("%s: found solution: [%lf, %lf, %lf, %lf, %lf]", 
        action_name_.c_str(), joints[0], joints[1], joints[2], joints[3],
        joints[4]);
      youbot_arm_joints::JointGoal goal;
      goal.timeout = 20.0;
      goal.name = "arm";
      goal.joints = joints;
      client_->sendGoal(goal);
      is_sent_goal_ = true;
      return ACTIVE;
      //ROS_INFO("Waitting for result ...");
    }
    if( client_->getState().toString() == "SUCCEEDED" ){   
       is_sent_goal_ = false;
       return SUCCEEDED;
    }
    else if( client_->getState().toString() == "ABORTED"){
       is_sent_goal_ = false;
       return ABORTED;
    }
    else return ACTIVE;
  }*/


  int isGoalValid( youbot_arm_ik::IkGoal goal){
    ROS_INFO("%s: accept goal", action_name_.c_str());

    if( isnan(goal.timeout) ||
        isnan(goal.duration) ||
        isnan(goal.id) ||
        isnan(goal.pitch) ||
        isnan(goal.position.x) ||
        isnan(goal.position.y) ||
        isnan(goal.position.z) ||
        isnan(goal.orientation.x) ||
        isnan(goal.orientation.y) ||
        isnan(goal.orientation.z) ) return 0;
    ROS_INFO("Goal: %.3lf, %.3lf,%s, %u, %.3lf, (%.3lf,%.3lf,%.3lf), (%.3lf,%.3lf,%.3lf),%d,%d,%d,%d",
      goal.timeout, goal.duration, goal.name.c_str(), 
      goal.id, goal.pitch, 
      goal.position.x, goal.position.y, goal.position.z,
      goal.orientation.x, goal.orientation.y, goal.orientation.z,
      goal.arm_to_front, goal.arm_bended_up, goal.gripper_downwards, 
      goal.move_arm);
    return 1;
  }
  
   void resetState(){
    is_sent_goal_ = false;
    time_start_ = ros::Time::now();
    unsubscribe();
    client_->cancelGoal();
  }

  void setAborted(youbot_arm_ik::IkResult result, std::string str = ""){
    ROS_ERROR("%s: Aborted, %s", action_name_.c_str(), str.c_str());
    result.solution = solution_;
    action_.setAborted(result);
    resetState();
  }

  void setSucceeded(youbot_arm_ik::IkResult result){
    ROS_INFO("%s: Succeeded, take %lf s", action_name_.c_str(),
      (time_now_ - time_start_).toSec());
    result.solution = solution_;
    action_.setSucceeded(result);
    resetState();
  }

  void setPreempted(){
    ROS_WARN("%s: Preempted", action_name_.c_str());
    action_.setPreempted();
    resetState();
  }

  void executeCB(const youbot_arm_ik::IkGoalConstPtr &goal_msg){
    youbot_arm_ik::IkGoal goal = *goal_msg;
    resetState();
    if( isGoalValid(goal) == 0 ){
      setAborted(result_, "Goal is invalid");
      return;
    }
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
            setAborted(result_, "Goal is invalid");
            return;
          }
          else {
            ROS_INFO("%s: accept new goal", action_name_.c_str());
            resetState();
          }
        }
        else{ setPreempted(); return;}
      }

      //control loop
      solution_.clear();
      int res_sol = 0;
      if(goal.name == "closest_ik"){ 
        subscribe();
        if(joints_state_.empty()){
          ROS_DEBUG("%s: Cannot receive joints state ", action_name_.c_str());
          continue;
        }
        res_sol = closestIK(goal,solution_);
      }
      else if(goal.name == "preferred_pitch_ik") 
        res_sol = preferredPitchIK(goal, solution_);
      else if(goal.name == "preferred_type_ik") 
        res_sol = preferredTypeIK(goal, solution_);
      else if(goal.name == "fully_constrained_ik") 
        res_sol = fullyConstrainedIK(goal, solution_);

      if(res_sol == 0){
        setAborted(result_,"No valid IK solution");
        return;
      }
      else{
        if(goal.move_arm){
          StateEnum state = moveArm(solution_, goal.duration);
          if( state == SUCCEEDED ) { setSucceeded(result_); return; }
          else if( state == ABORTED ) { setAborted(result_); return; }
        }
        else { setSucceeded(result_); return; }
      }
      r.sleep();
      if( r.cycleTime() > ros::Duration(1.0/rate_) )
        ROS_WARN("%s: Control desired rate of %.3lfHz... the loop actually took %.4lf seconds", action_name_.c_str(), rate_, r.cycleTime().toSec());
    }
  }

  int closestIK( youbot_arm_ik::IkGoal goal,
    std::vector<double>& sol_joints){
    boost::mutex::scoped_lock lock(mutex_);
    
    sol_joints.clear();
    solution_t current_joints;
    geometry_msgs::Point   goal_position;
    geometry_msgs::Vector3 goal_orientation;

    for(unsigned int i = 0; i < 5; ++i){
      current_joints.joints[i] = joints_state_[i];
    }
    goal_position    = goal.position;
    goal_orientation = goal.orientation;
    double roll = goal_orientation.x;
    roll = aricc_utils::smallAngle(roll);
    goal_orientation.x = 0.0;
    goal_orientation.z = 0.0;

    solution_t solution = YoubotGraspIK::solve_closest_ik(
      current_joints, goal_position, goal_orientation);
    
    if(!solution.feasible) return 0;
    //ROS_INFO("roll:%.3lf, joint[4]: %.3lf", roll, solution.joints[4]);
    solution.joints[4] += roll;
    //ROS_INFO("roll:%.3lf, joint[4]: %.3lf", roll, solution.joints[4]);
    for(unsigned int i = 0; i < 5; ++i)
      sol_joints.push_back(solution.joints[i]);
    return 1; 
  }
  
  int preferredPitchIK( youbot_arm_ik::IkGoal goal,
    std::vector<double>& sol){
    sol.clear();
    solution_t current_joints;
    geometry_msgs::Point   goal_position;
    geometry_msgs::Vector3 goal_orientation;
    double pitch = goal.pitch;

    goal_position    = goal.position;
    goal_orientation = goal.orientation;
    double roll = goal_orientation.x;
    roll = aricc_utils::smallAngle(roll);
    goal_orientation.x = 0.0;
    goal_orientation.z = 0.0;

    solution_t solution = YoubotGraspIK::solve_preferred_pitch_ik(
      pitch, goal_position, goal_orientation);
    
    if(!solution.feasible) return 0;
    //solution.joints[4] = 2.93 + roll;
    solution.joints[4] += roll;
    for(unsigned int i = 0; i < 5; ++i){
      sol.push_back(solution.joints[i]);
    }
    return 1; 
  }

  int preferredTypeIK( youbot_arm_ik::IkGoal goal,
    std::vector<double>& sol_joints){
    sol_joints.clear();
    geometry_msgs::Point   goal_position;
    geometry_msgs::Vector3 goal_orientation;

    goal_position    = goal.position;
    goal_orientation = goal.orientation;
    bool arm_to_front = goal.arm_to_front;
    bool arm_bended_up = goal.arm_bended_up;
    bool gripper_downwards = goal.gripper_downwards;

    solution_t solution = YoubotGraspIK::solve_preferred_type_ik(
      arm_to_front, arm_bended_up, gripper_downwards,
      goal_position, goal_orientation);
    
    if(!solution.feasible) return 0;
    for(unsigned int i = 0; i < 5; ++i)
      sol_joints.push_back(solution.joints[i]);
    return 1; 
  }
  
  int fullyConstrainedIK( youbot_arm_ik::IkGoal goal,
    std::vector<double>& sol_joints){
    sol_joints.clear();
    geometry_msgs::Point   goal_position;
    geometry_msgs::Vector3 goal_orientation;

    goal_position    = goal.position;
    goal_orientation = goal.orientation;
    int id = goal.id;
    double pitch = goal.pitch;

    solution_t solution = YoubotGraspIK::solve_fully_constrained_ik(
      id, pitch, goal_position, goal_orientation);
    
    if(!solution.feasible) return 0;
    for(unsigned int i = 0; i < 5; ++i)
      sol_joints.push_back(solution.joints[i]);
    return 1; 
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "youbot_arm_ik_server");
  IkAction action(ros::this_node::getName());
  ros::spin();
  return 0;
}
