#include <ros/ros.h>
#include "dynamic_reconfigure/server.h"
#include <actionlib/server/simple_action_server.h>
#include <youbot_base_local_move/BaseAction.h>
#include <youbot_base_local_move/ServerConfig.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <aricc_utils/geometry_utils.h>
#include <aricc_pid/pid_controller.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <utility>
#include <iostream>
#include <assert.h>

class BaseAction{

public:
  typedef youbot_base_local_move::ServerConfig Config;

protected:
  ros::NodeHandle nh_, pnh_;
  actionlib::SimpleActionServer
    <youbot_base_local_move::BaseAction> action_;
  std::string action_name_;
  boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
  
  geometry_msgs::Pose2D odom_now_;
  geometry_msgs::Pose2D odom_last_;
  geometry_msgs::Pose2D odom_start_;
  geometry_msgs::Pose2D odom_dist_;

  double rate_;
  boost::mutex mutex_;
  nav_msgs::Odometry odom_msg_;
  bool is_new_goal_;
  double linear_tolerance_, angular_tolerance_;

  double p_x_, i_x_, d_x_, i_min_x_,i_max_x_;
  double p_y_, i_y_, d_y_, i_min_y_,i_max_y_;
  double p_th_, i_th_, d_th_, i_min_th_,i_max_th_;
  double vx_min_, vx_max_;
  double vy_min_, vy_max_;
  double vth_min_, vth_max_;
  
  youbot_base_local_move::BaseFeedback feedback_;
  youbot_base_local_move::BaseResult result_;
 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Time time_now_, time_start_, time_last_; 
  aricc_control::PidController pid_x_, pid_y_, pid_th_;
 
public:
  BaseAction(std::string name) :
    pnh_("~"),is_new_goal_(true),
    action_(nh_, name, boost::bind(&BaseAction::executeCB, this, _1), false),
    action_name_(name){
    ROS_INFO("----------"); 
    pub_ = nh_.advertise<geometry_msgs::Twist >("cmd_vel", 1);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&BaseAction::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pnh_.param<double>("rate", rate_, 20.0);

    pnh_.param<double>("linear_tolerance", linear_tolerance_, 0.005);
    pnh_.param<double>("angular_tolerance", angular_tolerance_, 5); //In degree
    angular_tolerance_ = angular_tolerance_/180.0*M_PI;

    pnh_.param<double>("p_x", p_x_, 20);
    pnh_.param<double>("i_x", i_x_, 20);
    pnh_.param<double>("d_x", d_x_, 20);
    pnh_.param<double>("i_min_x", i_min_x_, 20);
    pnh_.param<double>("i_max_x", i_max_x_, 20);

    pnh_.param<double>("p_y", p_y_, 20);
    pnh_.param<double>("i_y", i_y_, 20);
    pnh_.param<double>("d_y", d_y_, 20);
    pnh_.param<double>("i_min_y", i_min_y_, 20);
    pnh_.param<double>("i_max_y", i_max_y_, 20);
    
    pnh_.param<double>("p_th", p_th_, 20);
    pnh_.param<double>("i_th", i_th_, 20);
    pnh_.param<double>("d_th", d_th_, 20);
    pnh_.param<double>("i_min_th", i_min_th_, 20);
    pnh_.param<double>("i_max_th", i_max_th_, 20);
    
    pnh_.param<double>("vx_max", vx_max_, 0.4);
    pnh_.param<double>("vx_min", vx_min_, 0.05);
    pnh_.param<double>("vy_max", vy_max_, 0.4);
    pnh_.param<double>("vy_min", vy_min_, 0.05);
    pnh_.param<double>("vth_max", vth_max_, 0.3);
    pnh_.param<double>("vth_min", vth_min_, 0.05);
    
    
    pid_x_.initPid(p_x_, i_x_, d_x_, i_max_x_, i_min_x_);
    pid_y_.initPid(p_y_, i_y_, d_y_, i_max_y_, i_min_y_);
    pid_th_.initPid(p_th_, i_th_, d_th_, i_max_th_, i_min_th_);

    action_.start();
    ROS_INFO("Starting %s ...", name.c_str());
  }

  ~BaseAction(void){
    sub_.shutdown();
  }

  void subscribe(){
    sub_ = nh_.subscribe("/odom"  ,1, &BaseAction::odomCB,this);
  }

  void unsubscribe(){
    sub_.shutdown();
  }

  void configCallback(Config &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    linear_tolerance_ = config.linear_tolerance;
    angular_tolerance_ = config.angular_tolerance;
    angular_tolerance_ = aricc_utils::deg2Rad(angular_tolerance_);
    p_x_ = config.p_x;
    i_x_ = config.i_x;
    d_x_ = config.d_x;
    i_min_x_ = config.i_min_x;
    i_max_x_ = config.i_max_x;
    
    p_y_ = config.p_y;
    i_y_ = config.i_y;
    d_y_ = config.d_y;
    i_min_y_ = config.i_min_y;
    i_max_y_ = config.i_max_y;
  
    p_th_ = config.p_th;
    i_th_ = config.i_th;
    d_th_ = config.d_th;
    i_min_th_ = config.i_min_th;
    i_max_th_ = config.i_max_th;

    vx_min_ = config.vx_min;
    vx_max_ = config.vx_max;
    vy_min_ = config.vy_min;
    vy_max_ = config.vy_max;
    vth_min_ = config.vth_min;
    vth_max_ = config.vth_max;
    
    pid_x_.setGains(p_x_, i_x_, d_x_, i_max_x_, i_min_x_);
    pid_y_.setGains(p_y_, i_y_, d_y_, i_max_y_, i_min_y_);
    pid_th_.setGains(p_th_, i_th_, d_th_, i_max_th_, i_min_th_);
  }

  void odomCB(const nav_msgs::Odometry::ConstPtr& msg){
    boost::mutex::scoped_lock lock(mutex_);
    odom_msg_ = *msg;
  }

  void stopBase(){
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
    pub_.publish(vel);
  }

  void moveBase(geometry_msgs::Twist vel){
    pub_.publish(vel);
  }

  double getPidVel(aricc_control::PidController pid,
    double cur_pos, double dest_pos, ros::Time now, ros::Time last){
    ros::Duration time_diff = now - last;
    //ROS_INFO("Dt:%.3lf", time_diff.toSec());
    double pos_diff = cur_pos - dest_pos;
    double vel = pid.updatePid( pos_diff, time_diff.toBoost());
    //double pp, ii, dd, i_max, i_min;
    //pid.getGains(pp,ii,dd,i_max, i_min);
    //ROS_INFO("PID:%.3lf,%.3lf,%.3lf,%.3lf,%.3lf", pp, ii, dd, i_max, i_min);
    //ROS_INFO("Cur:%.3lf, Dst:%.3lf,Vel:%.3lf", cur_pos, dest_pos, vel);
    return vel;
  }

  void rotatePose2D(double &vx, double &vy, double theta){
    Eigen::Vector3f vel_src;
    Eigen::Vector3f vel_dst;
    Eigen::Matrix3f rot_matrix_z;
    vel_src << vx,vy,0;
    rot_matrix_z << cos(theta), -sin(theta), 0,
                    sin(theta), cos(theta), 0,
                    0, 0, 1; 
    vel_dst = rot_matrix_z * vel_src;
    vx = vel_dst[0];
    vy = vel_dst[1];
  }

  void getVelocity(geometry_msgs::Twist &vel, geometry_msgs::Pose2D cur,
    geometry_msgs::Pose2D goal){
    ros::Time time_now = ros::Time::now();
    double vx =  getPidVel(pid_x_,  cur.x, goal.x, time_now, time_last_);
    double vy =  getPidVel(pid_y_,  cur.y, goal.y, time_now, time_last_ );
    double vth = getPidVel(pid_th_, cur.theta, goal.theta, time_now, time_last_ );
    time_last_ = time_now;
    
    //ROS_INFO("Vel: %.3lf, %.3lf, %.3lf", vx, vy, vth);

    double diff = fabs(cur.x - goal.x);    
    if( diff >= linear_tolerance_ ){
      if( fabs(vx)  > vx_max_ ) vx = aricc_utils::sign(vx)*vx_max_;
      if( fabs(vx)  < vx_min_ ) vx = aricc_utils::sign(vx)*vx_min_;
    }
    else vx = 0;
    
    diff = fabs(cur.y - goal.y);
    if( diff >= linear_tolerance_ ){
      if( fabs(vy)  > vy_max_ ) vy = aricc_utils::sign(vy)*vy_max_;
      if( fabs(vy)  < vy_min_ ) vy = aricc_utils::sign(vy)*vy_min_;
    }
    else vy = 0;

    diff = fabs(cur.theta - goal.theta);
    if(diff >= angular_tolerance_){
      if( fabs(vth)  > vth_max_ ) vth = aricc_utils::sign(vth)*vth_max_;
      if( fabs(vth)  < vth_min_ ) vth = aricc_utils::sign(vth)*vth_min_;
    }
    else vth = 0;
    rotatePose2D(vx, vy, -cur.theta );
    //ROS_INFO("Vel: %.3lf, %.3lf, %.3lf", vx, vy, vth);
    //ROS_INFO("vx,vy,vth:%.3lf, %.3lf, %.3lf", vx, vy, vth);   
    //ROS_INFO("----------");
 
    vel.linear.x = vx;
    vel.linear.y = vy;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = vth;
  }

  bool updateOdom(){
    boost::mutex::scoped_lock lock(mutex_);
    ros::Time now = ros::Time::now();
    double time_diff = ( now - odom_msg_.header.stamp ).toSec();
    if( time_diff > 0.5 ){
      //ROS_WARN("Updating Odom: msg is older %.3lf than now",time_diff);
      return false;
    }    

    double roll, pitch, yaw;
    aricc_utils::quat2Euler(odom_msg_.pose.pose.orientation, roll, pitch, yaw);
    odom_now_.x = odom_msg_.pose.pose.position.x;
    odom_now_.y = odom_msg_.pose.pose.position.y;
    odom_now_.theta = yaw;

    double diff_x, diff_y, diff_th;
    diff_x  = odom_now_.x - odom_last_.x;
    diff_y  = odom_now_.y - odom_last_.y;
    diff_th = odom_now_.theta - odom_last_.theta;

    //odom_dist_.x = odom_now_.x - odom_start_.x;
    //odom_dist_.y = odom_now_.y - odom_start_.y;

    rotatePose2D( diff_x, diff_y, -odom_start_.theta );
    if(diff_x < 1.0) odom_dist_.x += diff_x;
    if(diff_y < 1.0) odom_dist_.y += diff_y;
    if(fabs(diff_th) < M_PI) odom_dist_.theta += diff_th;
    if(fabs(odom_dist_.theta) > M_PI*2.0) odom_dist_.theta = 0.0;
    //ROS_INFO("odom:(%.3lf,%.3lf,%.3lf)",
    //  odom_dist_.x, odom_dist_.y, aricc_utils::rad2Deg(odom_dist_.theta));

    odom_last_ = odom_now_;
    return true; 
  }

  bool isReached( geometry_msgs::Pose2D goal ){
    if(!updateOdom()) return false;
    geometry_msgs::Twist vel_msg;
    getVelocity( vel_msg, odom_dist_, goal );
    moveBase(vel_msg);

    //Checking the goal is reached or not
    double diff = odom_dist_.x - goal.x;
    //ROS_INFO("diff_x:%.3lf", diff);
    if(fabs(diff) >= linear_tolerance_) return false;
    diff = odom_dist_.y - goal.y;
    //ROS_INFO("diff_y:%.3lf", diff);
    if(fabs(diff) >= linear_tolerance_) return false;
    diff = odom_dist_.theta - goal.theta;
    if(fabs(diff) >= angular_tolerance_) return false;
    
    stopBase(); 
    return true;
  }

  void resetOdom(geometry_msgs::Pose2D& odom){
    odom.x = 0.0;
    odom.y = 0.0;
    odom.theta = 0.0;
  }

  void resetState(){
    stopBase();
    is_new_goal_ = true;
    //time_start_ = ros::Time::now();
    unsubscribe();
    pid_x_.reset(); 
    pid_y_.reset(); 
    pid_th_.reset();
    resetOdom(odom_dist_);
    resetOdom(odom_start_);
    resetOdom(odom_last_);
    resetOdom(odom_now_);
  }

  bool initState(){
    subscribe();
    boost::mutex::scoped_lock lock(mutex_);
    //ROS_INFO("Start initializing State ...");
    ros::Time now = ros::Time::now();
    double time_diff = ( now - odom_msg_.header.stamp ).toSec();
    if( time_diff > 0.5 ){
      //ROS_WARN("Odom msg is older %.3lf than now",time_diff);
      return false;
    }

    double roll, pitch, yaw;
    aricc_utils::quat2Euler(odom_msg_.pose.pose.orientation, roll, pitch, yaw);
    odom_now_.x = odom_msg_.pose.pose.position.x;
    odom_now_.y = odom_msg_.pose.pose.position.y;
    odom_now_.theta = yaw;

    odom_start_  = odom_now_; 
    odom_last_   = odom_now_; 
    time_last_   = ros::Time::now();
    time_start_  = ros::Time::now();
    ROS_INFO("Finish initializing state ...");
    return true;
  }
  
  int isGoalValid( youbot_base_local_move::BaseGoal goal){
    ROS_INFO("%s: accept goal", action_name_.c_str());
    ROS_INFO("Goal: %.3lf, (%.3lf,%.3lf,%.3lf)",goal.timeout, 
      goal.destination.x, goal.destination.y, goal.destination.theta);
    if( isnan(goal.timeout)       ||
        isnan(goal.destination.x) || 
        isnan(goal.destination.y) || 
        isnan(goal.destination.theta) ||
        isinf(goal.timeout) ||
        isinf(goal.destination.x) ||
        isinf(goal.destination.y) ||
        isinf(goal.destination.theta)){
      ROS_WARN("Goal has NAN/INF item");
      return 0;
    }
    return 1;
  }

  void setAborted(youbot_base_local_move::BaseResult result, 
    std::string str = ""){
    ROS_ERROR("%s: Aborted, %s", action_name_.c_str(), str.c_str());
    action_.setAborted(result);
    resetState();
  }

  void setSucceeded(youbot_base_local_move::BaseResult result){
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
  
  void executeCB(const youbot_base_local_move::BaseGoalConstPtr &goal_msg){
    youbot_base_local_move::BaseGoal goal = *goal_msg;
    resetState();

    if( isGoalValid(goal) == 0 ){
      setAborted(result_,"Goal is invalid");
      return;
    }
    
    ros::Rate r(rate_);
    while(action_.isActive() || ros::ok()){
      r.sleep();
      if( r.cycleTime() > ros::Duration(1.0/rate_) )
        ROS_WARN("%s: Control desired rate of %.3lfHz... the loop actually took %.4lf seconds", action_name_.c_str(), rate_, r.cycleTime().toSec());
      
      if( is_new_goal_ ) {
        if( initState() ) is_new_goal_ = false;
        continue;
      }

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
          }
        }
        else{ setPreempted(); return; }
      }
      if(is_new_goal_) continue;
      if( isReached(goal.destination) ){ setSucceeded(result_); return; }
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "youbot_base_local_move_server");
  BaseAction action(ros::this_node::getName());
  ros::spin();
  return 0;
}
