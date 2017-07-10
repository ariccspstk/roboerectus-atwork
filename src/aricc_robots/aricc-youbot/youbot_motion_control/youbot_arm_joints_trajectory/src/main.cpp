#include <iostream>
#include <assert.h>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "brics_actuator/JointPositions.h"
#include "brics_actuator/JointVelocities.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <youbot_arm_joints_trajectory/TrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <utility>

class TrajectoryAction{

public:
   typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;

protected:
  ros::NodeHandle nh_, pnh_;
  actionlib::SimpleActionServer
    <youbot_arm_joints_trajectory::TrajectoryAction> action_;
  boost::shared_ptr<ActionClient> client_;
  std::string action_name_;
 
  sensor_msgs::JointState joint_msg_; 
  std::vector< std::pair<double, double> >joints_limit_;
  bool is_arm_pub_;
  bool is_sub_;
  double rate_;
  boost::mutex mutex_;

  youbot_arm_joints_trajectory::TrajectoryFeedback feedback_;
  youbot_arm_joints_trajectory::TrajectoryResult result_;
 
  ros::Subscriber sub_;
  ros::Time time_now_, time_start_; 
  ros::Time time_last_rev_;
  control_msgs::FollowJointTrajectoryGoal goal_sent_; 
 
public:
  TrajectoryAction(std::string name) :
    pnh_("~"),
    action_(nh_, name, boost::bind(&TrajectoryAction::executeCB, this, _1), false),
    action_name_(name), is_arm_pub_(false), is_sub_(false){
    ROS_INFO("----------"); 
    if(loadJointsLimit() == 0) return;
    pnh_.param<double>("rate", rate_, 20);
    client_.reset( new ActionClient("arm_1/arm_controller/follow_joint_trajectory", true) );
    if(!client_->waitForServer(ros::Duration(5.0))){
      ROS_ERROR("youbot arm_1/arm_controller/follow_joint_trajectory is not excuting");
    }
    action_.start();
    ROS_INFO("Starting %s ...", name.c_str());
  }

  ~TrajectoryAction(void){
  }

  void subscribe(){
    if(!is_sub_) {
      is_sub_ = true;
      sub_ = nh_.subscribe("/joint_states"  ,1, &TrajectoryAction::jointsCB,this);
      }
    }

  void unsubscribe(){
    sub_.shutdown();
    is_sub_ = false;
  }
  
  int loadJointsLimit(){
    ROS_INFO("Loading joints limit ...");
    try{
      XmlRpc::XmlRpcValue list;
      if( !pnh_.hasParam("joint_limit") ){
        ROS_ERROR("Cannot find joint limit list");
        return 0;
      }
      pnh_.getParam("joint_limit", list);
      ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_INFO("Found list size:%d", list.size());
      joints_limit_.clear();
      for(size_t i = 0; i < list.size(); ++i){
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string min = static_cast<std::string>(list[i]);
        ROS_ASSERT(list[++i].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string max = static_cast<std::string>(list[i]);
        //ROS_INFO("[MIN:%s, MAX:%s]",min.c_str(), max.c_str());
        std::pair<double, double> limit = std::make_pair( 
          atof(min.c_str()), atof(max.c_str()) ); 
        joints_limit_.push_back(limit);
        ROS_INFO("[MIN:%lf, MAX:%lf]", joints_limit_.at(i/2).first,
          joints_limit_.at(i/2).second);
      }
    }
    catch(ros::Exception e){
      ROS_ERROR("%s", e.what());
      return 0;
    }
    return 1;
  }
  
  void jointsCB(const sensor_msgs::JointState::ConstPtr& msg){
    boost::mutex::scoped_lock lock(mutex_);
    joint_msg_ = *msg; 
  }

  void printTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> traj){
    ROS_INFO("Trajectory has %lu points.", traj.size());
    for(unsigned int i = 0; i < traj.size(); ++i){
      std::ostringstream strs;
      std::string str = "Point_";
      strs << i << ": ";
      for(unsigned int j = 0; j < traj[i].positions.size(); ++j){
        strs << traj[i].positions[j] << " ";
      }
      strs << "Duration:" << traj[i].time_from_start;
      str += strs.str();
      ROS_INFO("%s", str.c_str());
    }
  }
  
  bool isArmReached(  std::vector<trajectory_msgs::JointTrajectoryPoint> segments, double tolerance = 6.5/180.0*M_PI){ 
    boost::mutex::scoped_lock lock(mutex_);
    if(joint_msg_.position.empty()){ 
      //ROS_WARN("Arm joint msg is empty");
      return false;
    }
    double time_diff = 
      (ros::Time::now()-joint_msg_.header.stamp).toSec();
    if( time_diff > 0.5 ){ 
      //ROS_WARN("Arm joint msg is older %.3lf than now",time_diff);
      return false;
    }

    std::string key_1 = "arm_joint_";
    //std::string key_2 = "gripper_finger_joint_";

    for(std::vector<std::string>::iterator it = joint_msg_.name.begin(); it != joint_msg_.name.end()-2; ++it){
      std::size_t found_1 = it->rfind(key_1);
      //std::size_t found_2 = it->rfind(key_2);
      if( found_1 == std::string::npos ){
        ROS_WARN("Received joint state error");
        return false;  
      }
    }
  /*Checking hasSentGoal variable to make sure
    only sent the goal one time.
  */
    std::vector<double> joints_state;
    joints_state.clear();
    joints_state = joint_msg_.position;
 
    if(!is_arm_pub_){
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory.joint_names.resize(5);
      goal.trajectory.joint_names[0] = "arm_joint_1";
      goal.trajectory.joint_names[1] = "arm_joint_2";
      goal.trajectory.joint_names[2] = "arm_joint_3";
      goal.trajectory.joint_names[3] = "arm_joint_4";
      goal.trajectory.joint_names[4] = "arm_joint_5";

      trajectory_msgs::JointTrajectoryPoint current_point;
      ros::Duration time_from_start;

      //Always set the current position as first segment
      current_point.positions.resize(5);
      current_point.velocities.resize(5);
      current_point.accelerations.resize(5);
      current_point.time_from_start = ros::Duration(1.0);
      time_from_start += current_point.time_from_start;

      for( unsigned int j = 0; j < 5; ++j){
        current_point.positions[j] = joints_state[j];
        current_point.velocities[j] = 0.0;
        current_point.accelerations[j] = 0.001;
      }
      goal.trajectory.points.push_back(current_point);
      for ( unsigned int i = 0; i < segments.size(); ++i){
        time_from_start += segments[i].time_from_start;
        current_point.time_from_start = time_from_start;
        
        for( unsigned int j = 0; j < 5; ++j){
          current_point.positions[j] = segments[i].positions[j];
          current_point.velocities[j] = 0.0;
          current_point.accelerations[j] = 0.001;
        }
        goal.trajectory.points.push_back(current_point);
      }

      //ROS_INFO("Sending to robot");
      //printTrajectory(goal.trajectory.points);
      goal.trajectory.header.stamp = ros::Time::now();
      client_->sendGoal(goal);
      goal_sent_ = goal;
      is_arm_pub_ = true;
      return false;
    }

    std::vector< trajectory_msgs::JointTrajectoryPoint >::iterator it;
    it = goal_sent_.trajectory.points.end()-1;
    //ROS_INFO("Last point:");
    //printTrajectory(goal_sent_.trajectory.points);
    //Checking the goal is reached or not
    for( size_t i = 0; i < 5; ++i ){
      double diff = 
        it->positions[i]-joints_state[i];
      //ROS_INFO("Goal: %.3lf, Current:%.3lf, Diff: %.3lf", it->positions[i],joints_state[i],diff);
      if( fabs(diff) > tolerance ) return false;
    }
    return true;
  }

  void resetState(){
    is_arm_pub_     = false;
    time_start_ = ros::Time::now();
    unsubscribe();
  }
  
  int isGoalValid( youbot_arm_joints_trajectory::TrajectoryGoal goal){
    ROS_INFO("%s: accept goal", action_name_.c_str());
    if( isnan(goal.timeout) ||
        isinf(goal.timeout)  
       ){ 
      ROS_ERROR("timout is nan or inf");
      return 0;
    }
    //ROS_ERROR("timout:%.3lf", goal.timeout);


    if(goal.points.empty()){
      ROS_ERROR("%s: No trajectory received", action_name_.c_str());
      return 0;
    }
    //printTrajectory(goal.points);
    
    for(size_t i = 0; i < goal.points.size(); ++i){
      if(goal.points[i].positions.size() != 5){ 
        ROS_ERROR("arm goal should have 5 postions, but received %lu", goal.points[i].positions.size());
        return 0;
      }

      std::ostringstream strs;
      for(size_t j = 0; j < goal.points[i].positions.size(); ++j){
        if(isnan(goal.points[i].positions[j]) || isinf(goal.points[i].positions[j])) return 0;
        if( (goal.points[i].positions[j] < joints_limit_.at(j).first) ||
            (goal.points[i].positions[j] > joints_limit_.at(j).second) ) {
          ROS_ERROR("Segment_%lu: Joint_%lu: [%.3lf],out of limit. Should be [%.3lf, %.3lf]",
             i, j, goal.points[i].positions[j], joints_limit_.at(j).first,
             joints_limit_.at(j).second);
          return 0;
        }
      }
    }
    return 1;
  }

  void setAborted(youbot_arm_joints_trajectory::TrajectoryResult result, std::string str = ""){
    ROS_ERROR("%s: Aborted, %s", action_name_.c_str(), str.c_str());
    action_.setAborted(result);
    resetState();
  }

  void setSucceeded(youbot_arm_joints_trajectory::TrajectoryResult result){
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

  void executeCB(const youbot_arm_joints_trajectory::TrajectoryGoalConstPtr &goal_msg){
    youbot_arm_joints_trajectory::TrajectoryGoal goal = *goal_msg;
    resetState();
    if( isGoalValid(goal) == 0 ){
      setAborted(result_,"Goal is invalid");
      return;
    }
    time_start_ = ros::Time::now();
    ros::Rate r(rate_);
    subscribe();
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
            subscribe();
          }
        }
        else{ setPreempted(); return; }
      }
  
      if( isArmReached(goal.points) ){ setSucceeded(result_); return; }
      r.sleep();
      if( r.cycleTime() > ros::Duration(1.0/rate_) )
        ROS_WARN("%s: Control desired rate of %.3lfHz... the loop actually took %.4lf seconds", action_name_.c_str(), rate_, r.cycleTime().toSec());
    }
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "youbot_arm_joints_trajectory_server");
  TrajectoryAction action(ros::this_node::getName());
  ros::spin();
  return 0;
}
