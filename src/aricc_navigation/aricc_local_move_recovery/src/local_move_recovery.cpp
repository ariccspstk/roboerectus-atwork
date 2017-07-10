/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <local_move_recovery/local_move_recovery.h>
#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace costmap_2d;

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(local_move_recovery, LocalMoveRecovery, local_move_recovery::LocalMoveRecovery, nav_core::RecoveryBehavior)

namespace local_move_recovery {
LocalMoveRecovery::LocalMoveRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
  tf_(NULL), initialized_(false), world_model_(NULL) {} 

void LocalMoveRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;
    costmap_ = local_costmap_->getCostmap();

    footprint_spec_ = local_costmap_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);
    ROS_INFO("%lf, %lf",inscribed_radius_, circumscribed_radius_);

    //get some parameters from the parameter server
    ros::NodeHandle blp_nh("~/" + name_);

    //we'll simulate every degree by default
    std::string controller_frequency_param_name;
    if(!blp_nh.searchParam("frequency", controller_frequency_param_name))
        sim_period_ = 0.05;
    else{
      frequency_ = 0;
      blp_nh.param(controller_frequency_param_name, frequency_, 20.0);
      if(frequency_ > 0)
          sim_period_ = 1.0 / frequency_;
      else{
          ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
          sim_period_ = 0.05;
        }
      }
    ROS_INFO("Sim period is set to %.2f", sim_period_);

    blp_nh.param("acc_lim_x", acc_lim_x_, 1.0);
    blp_nh.param("acc_lim_y", acc_lim_y_, 1.0);
    blp_nh.param("acc_lim_th", acc_lim_th_, 1.0);

    blp_nh.param("max_vel_x", max_vel_x_, 0.3);
    blp_nh.param("min_vel_x", min_vel_x_, -0.3);
    blp_nh.param("max_vel_y", max_vel_y_, 0.3);
    blp_nh.param("min_vel_y", min_vel_y_, -0.3);
    blp_nh.param("max_vel_th", max_vel_th_, 0.3);
    blp_nh.param("min_vel_th", min_vel_th_, -0.3);
    blp_nh.param("min_in_place_vel_th", min_in_place_vel_th_, 0.4);
    blp_nh.param("max_trans_vel", max_trans_vel_, 1.0);
    blp_nh.param("min_trans_vel", min_trans_vel_, 0.1);
    blp_nh.param("max_rot_vel", max_rot_vel_, 0.8);
    blp_nh.param("min_rot_vel", min_rot_vel_, 0.1);
    blp_nh.param("angular_sim_granularity", angular_sim_granularity_, 0.025);
    blp_nh.param("sim_granularity", sim_granularity_, 0.025);

    blp_nh.param("vx_samples", vx_samples_, 10);
    blp_nh.param("vy_samples", vy_samples_, 10);
    blp_nh.param("vtheta_samples", vtheta_samples_, 6);

    blp_nh.param("target_dist", target_dist_, 1.0);
    blp_nh.param("tolerance", tolerance_, 0.1);
    blp_nh.param("sim_time", sim_time_, 1.0);

    blp_nh.param("occdist_scale", occdist_scale_, 0.2);

    blp_nh.param("reset_distance", reset_distance_, 3.0);
    std::vector<std::string> clearable_layers_default, clearable_layers;
    clearable_layers_default.push_back( std::string("obstacles") );
    blp_nh.param("layer_names", clearable_layers, clearable_layers_default);


    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());
    //for comanding the base
    vel_pub_ = blp_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    initialized_ = true;
    ROS_INFO("Local move recovery behavior initialized");
    ROS_INFO("vel_x:[%lf,%lf],vel_y:[%lf,%lf],vel_th:[%lf,%lf]",
    min_vel_x_,max_vel_x_, min_vel_y_, max_vel_y_, min_vel_th_, max_vel_th_);
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

LocalMoveRecovery::~LocalMoveRecovery(){
  delete world_model_;
}


 /**
   * create and score a trajectory given the current pose of the robot and selected velocities
   */
void LocalMoveRecovery::generateTrajectory(
      double x, double y, double theta,
      double vx, double vy, double vtheta,
      double vx_samp, double vy_samp, double vtheta_samp,
      double acc_x, double acc_y, double acc_theta,
      base_local_planner::Trajectory& traj) {
    
    //ROS_INFO("samp:%lf,%lf,%lf",vx_samp ,vy_samp, vtheta_samp);
    //current robot pose
    double x_i = x;
    double y_i = y;
    double theta_i = theta;

    double vx_i = vx;
    double vy_i = vy;
    double vtheta_i = vtheta;

    //compute the magnitude of the velocities
    double vmag = sqrt(vx_samp * vx_samp + vy_samp * vy_samp);
    double eps = 1e-4;
    max_trans_vel_ = max_vel_x_;

    //make sure that the velocity sample is within specific range
    //otherwise, the trajectory is invalid
    if( ( (vmag + eps) < min_trans_vel_ &&
         (fabs(vtheta_samp)+eps) < min_rot_vel_ ) ||
         (vmag - eps) > max_trans_vel_ ){
      traj.cost_ = -1.0;
      ROS_WARN("v_samp:[%lf,%lf,%lf]",vx_samp, vy_samp, vtheta_samp);
      ROS_WARN("Velocity Sample is out of range");
      return;
    }

    //compute the number of steps we must take along this trajectory to be "safe"
    int  num_steps = int(max((vmag * sim_time_) / sim_granularity_, fabs(vtheta_samp) / angular_sim_granularity_) + 0.5);

    //we at least want to take one step... even if we won't move, we want to score our current position
    if(num_steps == 0) num_steps = 1;

    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vx_samp;
    traj.yv_ = vy_samp;
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    //initialize the costs for the trajectory
    double occ_cost = 0.0;

    for(int i = 0; i < num_steps; ++i){

      //we don't want a path that goes off the know map
      unsigned int cell_x, cell_y;
      if(!costmap_->worldToMap(x_i, y_i, cell_x, cell_y)){
        ROS_WARN("Path goes off the know map %d/%d",i, num_steps);
        traj.cost_ = -1.0;
        return;
      }

      //if footprint hits an obstacle this trajectory is invalid
      double footprint_cost = footprintCost(x_i, y_i, theta_i);
      if(footprint_cost < 0){
        ROS_WARN("Robot hits obstacle %d/%d", i, num_steps);
        traj.cost_ = -1.0;
        return;
      }

      //ROS_WARN("%lf,%lf,%lf", occ_cost, footprint_cost, double(costmap_->getCost(cell_x, cell_y)));
      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_->getCost(cell_x, cell_y)));
      
    //the point is legal... add it to the trajectory
    traj.addPoint(x_i, y_i, theta_i);

    //calculate velocities
    vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
    vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
    vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

    //calculate positions
    x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
    y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
    theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

    //increment time
    time += dt;
  } // end for i < numsteps

  double cost = -1.0;

  //Calculate total cost
  cost = occdist_scale_ * occ_cost;
  traj.cost_ = cost;
  ROS_WARN("Traj Cost: %lf", traj.cost_);
}

//we need to take the footprint of the robot into account when we calculate cost to obstacles
double LocalMoveRecovery::footprintCost(double x_i, double y_i, double theta_i){
   return world_model_->footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
  }   

base_local_planner::Trajectory LocalMoveRecovery::createTrajectories(double x, double y, double theta,
      double vx, double vy, double vtheta,
      double acc_x, double acc_y, double acc_theta) {

  //compute feasible velocity limits in robot space
  double max_vel_x = 0, max_vel_y = 0, max_vel_theta = 0;
  double min_vel_x = 0, min_vel_y = 0, min_vel_theta = 0;

  max_vel_x = min(max_vel_x_, vx + acc_x * sim_period_);
  max_vel_y = min(max_vel_y_, vy + acc_y * sim_period_);
  max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_period_);

  min_vel_x = max(min_vel_x_, vx - acc_x * sim_period_);
  min_vel_y = max(min_vel_y_, vy - acc_y * sim_period_);
  min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_period_);
  ROS_INFO("acc: [%lf, %lf, %lf], sim_period:%lf", acc_x, acc_y, acc_theta, sim_period_);
  ROS_INFO("min_vel: [%lf, %lf, %lf]", min_vel_x, min_vel_y, min_vel_theta);

  //we want to sample the velocity space regularly
  double dvx = (max_vel_x - min_vel_x) / (vx_samples_ - 1);
  double dvy = (max_vel_y - min_vel_y) / (vy_samples_ - 1);
  double dvtheta = (max_vel_theta - min_vel_theta) / (vtheta_samples_ - 1);

  double vx_samp = min_vel_x;
  double vy_samp = min_vel_y;
  double vtheta_samp = min_vel_theta;

  ROS_INFO("dvx: [%lf], dvy: [%lf], dvtheta: [%lf]", dvx, dvy, dvtheta);
  ROS_INFO("v_samp: [%lf, %lf, %lf]", vx_samp, vy_samp, vtheta_samp);

 //keep track of the best trajectory seen so far
  base_local_planner::Trajectory* best_traj = &traj_one_;
  best_traj->cost_ = -1.0;

  base_local_planner::Trajectory* comp_traj = &traj_two_;
  comp_traj->cost_ = -1.0;

  ROS_INFO("current pose:%lf,%lf,%lf",x ,y, theta);
  ROS_INFO("current acc:%lf,%lf,%lf",acc_x ,acc_y, acc_theta);
  ROS_INFO("current vel:%lf,%lf,%lf",vx ,vy, vtheta);
  for(int i = 0; i < vx_samples_; ++i){
    //loop through all y velocities
    vy_samp = min_vel_y;
    for(int j = 0; j < vy_samples_; ++j){
      //loop through all theta velocities
      vtheta_samp = min_vel_theta;
      for(int k = 0; k < vtheta_samples_; ++k){
        //ROS_INFO("v_samp: [%lf, %lf, %lf]", vx_samp, vy_samp, vtheta_samp);
        generateTrajectory( x, y, theta,
                              vx, vy, vtheta,
                              vx_samp, vy_samp, vtheta_samp,
                              acc_x, acc_y, acc_theta,
                              *comp_traj);
        selectBestTrajectory(best_traj,comp_traj);
        vtheta_samp += dvtheta;
      }
      vy_samp += dvy;
    }
    vx_samp += dvx;
  }
  ROS_WARN("Best_traj:%lf",best_traj->cost_);
  ROS_WARN("Comp_traj:%lf",comp_traj->cost_);
  return *best_traj;
}

void LocalMoveRecovery::publishVel(double vx, double vy, double vth){

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = vx;
  cmd_vel.linear.y = vy;
  cmd_vel.angular.z = vth;
  ROS_INFO("Publish:%lf,%lf,%lf",vx,vy,vth);
  vel_pub_.publish(cmd_vel);
}

void LocalMoveRecovery::selectBestTrajectory(
  base_local_planner::Trajectory* &best_traj,
  base_local_planner::Trajectory* &comp_traj)
{
  base_local_planner::Trajectory* swap = NULL;
  if( comp_traj->cost_ >= 0 &&
      (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0))
  {
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;
  }
}

bool LocalMoveRecovery::isReachedGoal(double current_dist, double tolerance){
  double diff = fabs(current_dist - target_dist_);
  if(diff <= tolerance) return true;
  return false;

}

void LocalMoveRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the LocalMoveRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  
  ROS_WARN("Local Move Recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  tf::Stamped<tf::Pose> global_pose;
  //local_costmap_->getRobotPose(global_pose);
  global_costmap_->getRobotPose(global_pose);
  double current_dist = 0.0;
  double prev_x, prev_y;
  prev_x = global_pose.getOrigin().x();
  prev_y = global_pose.getOrigin().y();

  while(n.ok()){
    //Get current pose of robot
    global_costmap_->getRobotPose(global_pose);
    //local_costmap_->getRobotPose(global_pose);
    double x = global_pose.getOrigin().x();
    double y = global_pose.getOrigin().y();
    double theta =  tf::getYaw(global_pose.getRotation());
    ROS_INFO("POSE:%lf,%lf,%lf",x, y,theta);
    double dist = sqrt((prev_x - x)*(prev_x - x) + (prev_y-y)*(prev_y-y));
    current_dist += dist;
    prev_x = x;
    prev_y = y;

    //Check whether robot reaches the goal(distance)
      if(isReachedGoal(current_dist,tolerance_)) {
        publishVel(0,0,0);
        ROS_WARN("Local Move Recovery behavior reached goal.");
        return;
      }

    else{
      //Create trajectories
      ROS_WARN("Local Move Recovery behavior:creating trajectories.");
      tf::Stamped<tf::Pose> robot_vel;
      odom_helper_.getRobotVel(robot_vel);
  
      double vx = robot_vel.getOrigin().x();
      double vy = robot_vel.getOrigin().y();
      double vtheta = tf::getYaw(robot_vel.getRotation());
      
      base_local_planner::Trajectory best_traj; 
      best_traj = createTrajectories(x, y, theta, 
        vx, vy, vtheta, acc_lim_x_, acc_lim_y_, acc_lim_th_);
      if(best_traj.cost_ < 0) {
        ROS_WARN("No best trajectory found!");
        publishVel(0,0,0);
        return;
      }
      ROS_INFO("Got Trajectory: %lf, %lf, %lf", best_traj.xv_, best_traj.yv_, best_traj.thetav_);
      publishVel(best_traj.xv_, best_traj.yv_, best_traj.thetav_);
     }
 
    r.sleep();
  }
}
};
