/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include <mdp_base_local_planner/trajectory_planner.h>
#include <costmap_2d/footprint.h>
#include <string>
#include <sstream>
#include <math.h>
#include <angles/angles.h>
#include <boost/algorithm/string.hpp>
#include <ros/console.h>
//for computing path distance
#include <queue>

using namespace std;
using namespace costmap_2d;

namespace mdp_base_local_planner{

  void TrajectoryPlanner::reconfigure(CustBaseLocalPlannerConfig &cfg){
      CustBaseLocalPlannerConfig config(cfg);
      boost::mutex::scoped_lock l(configuration_mutex_);
      acc_lim_x_ = config.acc_lim_x;
      acc_lim_y_ = config.acc_lim_y;
      acc_lim_theta_ = config.acc_lim_theta;

      max_vel_x_ = config.max_vel_x;
      min_vel_x_ = -max_vel_x_;
      // min_vel_x_ = config.min_vel_x;
      
      max_vel_y_ = config.max_vel_y;
      min_vel_y_ = -max_vel_y_;
      //min_vel_y_ = config.min_vel_y;
      
      max_vel_th_ = config.max_vel_theta;
      min_vel_th_ = -max_vel_y_;
      //min_vel_th_ = config.min_vel_theta;
      min_in_place_vel_th_ = config.min_in_place_vel_theta;

      sim_time_ = config.sim_time;
      sim_granularity_ = config.sim_granularity;
      angular_sim_granularity_ = config.angular_sim_granularity;

      pdist_scale_ = config.pdist_scale;
      gdist_scale_ = config.gdist_scale;
      occdist_scale_ = config.occdist_scale;
      heading_scale_ = config.heading_scale;
      path_align_scale_ = config.path_align_scale;
      path_backward_scale_ = config.path_backward_scale;

      if (meter_scoring_) {
        //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
        double resolution = costmap_.getResolution();
        gdist_scale_ *= resolution;
        pdist_scale_ *= resolution;
        occdist_scale_ *= resolution;
      }

      oscillation_reset_dist_ = config.oscillation_reset_dist;

      vx_samples_ = config.vx_samples;
      vy_samples_ = config.vy_samples;
      vtheta_samples_ = config.vtheta_samples;

      if (vx_samples_ <= 0) {
          config.vx_samples = 1;
          vx_samples_ = config.vx_samples;
          ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
      }
      if (vy_samples_ <= 0) {
          config.vy_samples = 1;
          vy_samples_ = config.vy_samples;
          ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
      }
      if(vtheta_samples_ <= 0) {
          config.vtheta_samples = 1;
          vtheta_samples_ = config.vtheta_samples;
          ROS_WARN("You've specified that you don't want any samples in the theta dimension. We'll at least assume that you want to sample one value... so we're going to set vtheta_samples to 1 instead");
      }

      heading_lookahead_ = config.heading_lookahead;

      //x-vels
      string x_string = config.x_vels;
      vector<string> x_strs;
      boost::split(x_strs, x_string, boost::is_any_of(", "), boost::token_compress_on);

      vector<double>x_vels;
      for(vector<string>::iterator it=x_strs.begin(); it != x_strs.end(); ++it) {
          istringstream iss(*it);
          double temp;
          iss >> temp;
          x_vels.push_back(temp);
          //ROS_INFO("Adding x_vel: %e", temp);
      }
      x_vels_ = x_vels;

      //y-vels
      string y_string = config.y_vels;
      vector<string> y_strs;
      boost::split(y_strs, y_string, boost::is_any_of(", "), boost::token_compress_on);

      vector<double>y_vels;
      for(vector<string>::iterator it=y_strs.begin(); it != y_strs.end(); ++it) {
          istringstream iss(*it);
          double temp;
          iss >> temp;
          y_vels.push_back(temp);
          //ROS_INFO("Adding y_vel: %e", temp);
      }

      y_vels_ = y_vels;
      
      //theta-vels
      string th_string = config.th_vels;
      vector<string> th_strs;
      boost::split(th_strs, th_string, boost::is_any_of(", "), boost::token_compress_on);

      vector<double>th_vels;
      for(vector<string>::iterator it=th_strs.begin(); it != th_strs.end(); ++it) {
          istringstream iss(*it);
          double temp;
          iss >> temp;
          th_vels.push_back(temp);
          //ROS_INFO("Adding th_vel: %e", temp);
      }
      th_vels_ = th_vels;
  }

  TrajectoryPlanner::TrajectoryPlanner(
      base_local_planner::WorldModel& world_model, 
      const Costmap2D& costmap, 
      std::vector<geometry_msgs::Point> footprint_spec,
      double acc_lim_x, 
      double acc_lim_y, 
      double acc_lim_theta,
      double sim_time, 
      double sim_granularity, 
      int vx_samples, 
      int vy_samples, 
      int vtheta_samples,
      double pdist_scale, 
      double gdist_scale, 
      double occdist_scale,
      double heading_scale,
      double heading_lookahead, 
      double path_align_scale,
      double path_backward_scale,
      double oscillation_reset_dist,
      double max_vel_x, 
      double min_vel_x,
      double max_vel_y, 
      double min_vel_y,
      double max_vel_th, 
      double min_vel_th, 
      double min_in_place_vel_th,
      double min_vel_x_y,
      double min_rot_vel,
      double dist_rotate_to_goal,
      bool meter_scoring,
      vector<double> x_vels,
      vector<double> y_vels, 
      vector<double> th_vels, 
      double stop_time_buffer, 
      double sim_period, 
      double angular_sim_granularity)
    : path_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
      goal_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
    costmap_(costmap),
    world_model_(world_model), 
    footprint_spec_(footprint_spec),
    sim_time_(sim_time), 
    sim_granularity_(sim_granularity), 
    angular_sim_granularity_(angular_sim_granularity),
    vx_samples_(vx_samples), 
    vy_samples_(vy_samples),
    vtheta_samples_(vtheta_samples),
    pdist_scale_(pdist_scale), 
    gdist_scale_(gdist_scale), 
    occdist_scale_(occdist_scale),
    heading_scale_(heading_scale),
    path_align_scale_(path_align_scale),
    path_backward_scale_(path_backward_scale),
    acc_lim_x_(acc_lim_x), 
    acc_lim_y_(acc_lim_y), 
    acc_lim_theta_(acc_lim_theta),
    prev_x_(0), 
    prev_y_(0), 
    heading_lookahead_(heading_lookahead), 
    oscillation_reset_dist_(oscillation_reset_dist), 
    max_vel_x_(max_vel_x), 
    min_vel_x_(min_vel_x), 
    max_vel_y_(max_vel_y), 
    min_vel_y_(min_vel_y), 
    max_vel_th_(max_vel_th), 
    min_vel_th_(min_vel_th), 
    min_in_place_vel_th_(min_in_place_vel_th),
    min_vel_x_y_(min_vel_x_y),
    min_rot_vel_(min_rot_vel),
    dist_rotate_to_goal_(dist_rotate_to_goal),
    meter_scoring_(meter_scoring),
    x_vels_(x_vels),
    y_vels_(y_vels), 
    th_vels_(th_vels),
    stop_time_buffer_(stop_time_buffer), 
    sim_period_(sim_period)
  {
    //the robot is not stuck to begin with
    resetOscillationFlags();
    final_goal_position_valid_ = false;

    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);

    ROS_INFO("Aricc MDP Trajectory Planner initilized");
  }

  TrajectoryPlanner::~TrajectoryPlanner(){}

  bool TrajectoryPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
    base_local_planner::MapCell cell = path_map_(cx, cy);
    base_local_planner::MapCell goal_cell = goal_map_(cx, cy);
    if (cell.within_robot) {
        return false;
    }
    occ_cost = costmap_.getCost(cx, cy);
    if (cell.target_dist == path_map_.obstacleCosts() ||
        cell.target_dist == path_map_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
    }
    path_cost = cell.target_dist;
    goal_cost = goal_cell.target_dist;
    total_cost = pdist_scale_ * path_cost + gdist_scale_ * goal_cost + occdist_scale_ * occ_cost;
    return true;
  }

  /**
   * create and score a trajectory given the current pose of the robot and selected velocities
   */
  void TrajectoryPlanner::generateTrajectory(
      double x, double y, double theta,
      double vx, double vy, double vtheta,
      double vx_samp, double vy_samp, double vtheta_samp,
      double acc_x, double acc_y, double acc_theta,
      double impossible_cost,
      bool ahead_scoring,
      double dist_to_goal,
      base_local_planner::Trajectory& traj) {

    // make sure the configuration doesn't change mid run
    boost::mutex::scoped_lock l(configuration_mutex_);

    double x_i = x;
    double y_i = y;
    double theta_i = theta;

    double vx_i, vy_i, vtheta_i;

    vx_i = vx;
    vy_i = vy;
    vtheta_i = vtheta;

    //compute the magnitude of the velocities
    double vmag = sqrt(vx_samp * vx_samp + vy_samp * vy_samp);
    double eps = 1e-4;
    max_vel_x_y_ = max_vel_x_;
    

    if( ( (vmag + eps) < min_vel_x_y_ &&
         (fabs(vtheta_samp)+eps) < min_rot_vel_ ) ||
         (vmag - eps) > max_vel_x_y_ ||
        checkOscillationFlags(vx_samp,vy_samp,vtheta_samp)){
      //ROS_WARN("Got oscillation flags");
      traj.cost_ = -1.0;
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
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;
    double goal_heading_diff = 0.0;
    double traj_heading_diff = 0.0;

    for(int i = 0; i < num_steps; ++i){
      //we don't want a path that goes off the know map
      unsigned int cell_x, cell_y;
      if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
        //ROS_WARN("Path goes off the know map");
        traj.cost_ = -1.0;
        return;
      }

      //if footprint hits an obstacle this trajectory is invalid
      double footprint_cost = footprintCost(x_i, y_i, theta_i);
      if(footprint_cost < 0)
      {
        //ROS_WARN("Robot hits obstacle");
        traj.cost_ = -1.0;
        return;
      }

      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));

      //update path and goal distances
      path_dist = path_map_(cell_x, cell_y).target_dist;
      goal_dist = goal_map_(cell_x, cell_y).target_dist;
      
      //if a point on this trajectory has no clear path to goal it is invalid
      if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
        //ROS_WARN("No clear path to goal");
        traj.cost_ = -2.0;
        return;
      }

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
  

    //ROS_INFO("OccCost: %f, vx: %.2f, vy: %.2f, vtheta: %.2f", occ_cost, vx_samp, vy_samp, vtheta_samp);
    double cost = -1.0;

  //Calculate heading diffence between trajectory and global goal
  double goal_heading = tf::getYaw(global_plan_.back().pose.orientation);
  goal_heading_diff = fabs( angles::shortest_angular_distance(theta_i,goal_heading));

  //TODO scoring orientation of local trajectory and global trajectory
  unsigned int cell_cu_x, cell_cu_y;
  if(!costmap_.worldToMap(x_i,y_i,cell_cu_x,cell_cu_y)){
    traj.cost_ = -1.0;
    return;
  }
  traj_heading_diff = headingDiff(cell_cu_x,cell_cu_y,
                                  x_i,y_i,theta_i);
  if (traj_heading_diff > M_PI_2) traj_heading_diff -= M_PI;
  else if (traj_heading_diff < -M_PI_2) traj_heading_diff += M_PI;
  traj_heading_diff = fabs(traj_heading_diff);
  
  //double traj_heading = tf::getYaw(global_plan_.front().pose.orientation);
  //traj_heading_diff = fabs( angles::shortest_angular_distance(theta_i,traj_heading));
  
  //ROS_INFO("Traj_heading_diff:%lf",traj_heading_diff*180.0/M_PI);
  
  double path_backward_scale = 0;
  if(traj.xv_ >= 0) path_backward_scale = 0;
  else path_backward_scale = path_backward_scale_;

  if(dist_to_goal > dist_rotate_to_goal_){
    cost = pdist_scale_ * path_dist + 
         gdist_scale_ * goal_dist + 
         occdist_scale_ * occ_cost +
         path_align_scale_ * traj.yv_ + 
         path_backward_scale * fabs(traj.xv_) +
         heading_scale_ * traj_heading_diff;
  }
  else{
    cost = pdist_scale_ * path_dist + 
         gdist_scale_ * goal_dist + 
         occdist_scale_ * occ_cost +
         path_align_scale_ * traj.yv_ + 
         path_backward_scale * fabs(traj.xv_) +
         heading_scale_ * goal_heading_diff;
  }


  //ROS_INFO("Scale:(%lf, %lf, %lf, %lf, %lf, %lf)",pdist_scale_, gdist_scale_, occdist_scale_, path_align_scale_, path_backward_scale_, heading_scale_);
  //Calculate cost of trajectory
  traj.cost_ = cost;
}

  double TrajectoryPlanner::headingDiff(int cell_x, int cell_y, double x, double y, double heading){
    double heading_diff = DBL_MAX;
    unsigned int goal_cell_x, goal_cell_y;
    //find a clear line of sight from the robot's cell to a point on the path
    for (int i = global_plan_.size() - 1; i >=0; --i) 
    {
      if (costmap_.worldToMap(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y, goal_cell_x, goal_cell_y)) 
      {
        if (lineCost(cell_x, goal_cell_x, cell_y, goal_cell_y) >= 0) {
          double gx, gy;
          costmap_.mapToWorld(goal_cell_x, goal_cell_y, gx, gy);
          double v1_x = gx - x;
          double v1_y = gy - y;
          double v2_x = cos(heading);
          double v2_y = sin(heading);

          double perp_dot = v1_x * v2_y - v1_y * v2_x;
          double dot = v1_x * v2_x + v1_y * v2_y;

          //get the signed angle
          double vector_angle = atan2(perp_dot, dot);

          heading_diff = fabs(vector_angle);
          return heading_diff;

        }
      }
    }
    return heading_diff;
  }

  //calculate the cost of a ray-traced line
  double TrajectoryPlanner::lineCost(int x0, int x1, 
      int y0, int y1){
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    double line_cost = 0.0;
    double point_cost = -1.0;

    if (x1 >= x0)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y0)                 // The y-values are increasing
    {
      yinc1 = 1;
      yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1 = -1;
      yinc2 = -1;
    }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    } else {                      // There is at least one y-value for every x-value
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++) {
      point_cost = pointCost(x, y); //Score the current point

      if (point_cost < 0) {
        return -1;
      }

      if (line_cost < point_cost) {
        line_cost = point_cost;
      }

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den) {           // Check if numerator >= denominator
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }

    return line_cost;
  }

  double TrajectoryPlanner::pointCost(int x, int y){
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION){
      return -1;
    }

    return cost;
  }

  void TrajectoryPlanner::updatePlan(const vector<geometry_msgs::PoseStamped>& new_plan, bool compute_dists){
    global_plan_.resize(new_plan.size());
    for(unsigned int i = 0; i < new_plan.size(); ++i){
      global_plan_[i] = new_plan[i];
    }

    if( global_plan_.size() > 0 ){
      geometry_msgs::PoseStamped& final_goal_pose = global_plan_[ global_plan_.size() - 1 ];
      final_goal_x_ = final_goal_pose.pose.position.x;
      final_goal_y_ = final_goal_pose.pose.position.y;
      final_goal_position_valid_ = true;
    } else {
      final_goal_position_valid_ = false;
    }

    if (compute_dists) 
   {
      //reset the map for new operations
      path_map_.resetPathDist();
      goal_map_.resetPathDist();

      //make sure that we update our path based on the global plan and compute costs
      path_map_.setTargetCells(costmap_, global_plan_);
      goal_map_.setLocalGoal(costmap_, global_plan_);
      ROS_DEBUG("Path/Goal distance computed");
    }
  }

  bool TrajectoryPlanner::checkTrajectory(double x, double y, double theta, double vx, double vy, 
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
    //ROS_INFO("Check trajectory");
    resetOscillationFlags();
    base_local_planner::Trajectory t; 

    double cost = scoreTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp);

    //if the trajectory is a legal one... the check passes
    if(cost >= 0) return true;
    //ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vx_samp, vy_samp, vtheta_samp, cost);

    //otherwise the check fails
    return false;
  }

  double TrajectoryPlanner::scoreTrajectory(double x, double y, double theta, double vx, double vy, 
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp) {
    base_local_planner::Trajectory t; 
    double impossible_cost = path_map_.obstacleCosts();
    generateTrajectory(x, y, theta,
                       vx, vy, vtheta,
                       vx_samp, vy_samp, vtheta_samp,
                       acc_lim_x_, acc_lim_y_, acc_lim_theta_,
                       impossible_cost, false,0,t);

    // return the cost.
    return double( t.cost_ );
  }

  /*
   * create the trajectories we wish to score
   */
  base_local_planner::Trajectory TrajectoryPlanner::createTrajectories(double x, double y, double theta, 
      double vx, double vy, double vtheta,
      double acc_x, double acc_y, double acc_theta) {
  //Calculate the distance between robot and goal
  double dist_to_goal =
         (x - global_plan_.back().pose.position.x) *
         (x - global_plan_.back().pose.position.x) + 
         (y - global_plan_.back().pose.position.y) * 
         (y - global_plan_.back().pose.position.y);
  dist_to_goal = sqrt(dist_to_goal);
  bool ahead_scoring = true;
  if(dist_to_goal < heading_lookahead_){
    ahead_scoring = false;
    //ROS_INFO("Scoring one point");
  }
  
  //compute feasible velocity limits in robot space
  double max_vel_x = 0, max_vel_y = 0, max_vel_theta = 0;
  double min_vel_x = 0, min_vel_y = 0, min_vel_theta = 0;

  max_vel_x = min(max_vel_x_, vx + acc_x * sim_period_);
  max_vel_y = min(max_vel_y_, vy + acc_y * sim_period_);
  max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_period_);
      
  min_vel_x = max(min_vel_x_, vx - acc_x * sim_period_);
  min_vel_y = max(min_vel_y_, vy - acc_y * sim_period_);
  min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_period_);
  
  //we want to sample the velocity space regularly
  double dvx = (max_vel_x - min_vel_x) / (vx_samples_ - 1);
  double dvy = (max_vel_y - min_vel_y) / (vy_samples_ - 1);
  double dvtheta = (max_vel_theta - min_vel_theta) / (vtheta_samples_ - 1);

  double vx_samp = min_vel_x;
  double vy_samp = min_vel_y;
  double vtheta_samp = min_vel_theta;

  //keep track of the best trajectory seen so far
  base_local_planner::Trajectory* best_traj = &traj_one;
  best_traj->cost_ = -1.0;

  base_local_planner::Trajectory* comp_traj = &traj_two;
  comp_traj->cost_ = -1.0;

  //ROS_WARN("max_vel: %lf, %lf, %lf", max_vel_x_, max_vel_y_, max_vel_th_);
  //ROS_WARN("min_vel: %lf, %lf, %lf", min_vel_x_, min_vel_y_, min_vel_th_);

  //any cell with a cost greater than the size of the map is impossible
  double impossible_cost = path_map_.obstacleCosts();
  //loop through all x,y,theta velocities
  
  for(int i = 0; i < vx_samples_; ++i){
    //loop through all y velocities
    vy_samp = min_vel_y;
    for(int j = 0; j < vy_samples_; ++j){
      //loop through all theta velocities
      vtheta_samp = min_vel_theta;
      for(int k = 0; k < vtheta_samples_; ++k){
        //if(vx_samp != 0 && vy_samp != 0 && vtheta_samp != 0)
        {
          generateTrajectory( x, y, theta, 
                              vx, vy, vtheta, 
                              vx_samp, vy_samp, vtheta_samp, 
                              acc_x, acc_y, acc_theta, 
                              impossible_cost, 
                              ahead_scoring, dist_to_goal,
                              *comp_traj);
          //if the new trajectory is better... let's take it
          selectBestTrajectory(best_traj,comp_traj);
        }
        vtheta_samp += dvtheta;
      }
      vy_samp += dvy;
    }
    vx_samp += dvx;
  }

  /*
  ROS_INFO("Oscillation_flags: stuck_forward:%d,stuck_backward:%d,stuck_left_strafe:%d,stuck_right_strafe:%d,stuck_left:%d,stuck_right:%d",stuck_forward_move, stuck_backward_move,stuck_left_strafe, stuck_right_strafe,stuck_left, stuck_right);*/

  //Try sample velocities
  /*
  vx_samp = -0.2;
  vy_samp = 0;
  vtheta_samp = 0;
  generateTrajectory( x, y, theta,
                              vx, vy, vtheta,
                              vx_samp, vy_samp, vtheta_samp,
                              acc_x, acc_y, acc_theta,
                              impossible_cost, *comp_traj);

  ROS_INFO("back trajectory :%lf,%lf,%lf,%lf",
                             comp_traj->cost_,comp_traj->xv_,
                             comp_traj->yv_,comp_traj->thetav_);
  selectBestTrajectory(best_traj,comp_traj);
  */

  /*
  for(int i = 0; i < x_vels_.size(); ++i)
  {
    for(int j = 0; j < y_vels_.size(); ++j)
    {
      for(int k = 0; k < th_vels_.size(); ++k)
      {
        vx_samp = x_vels_[i];
        vy_samp = y_vels_[j];
        vtheta_samp = th_vels_[k];
        //if(vx_samp != 0 && vy_samp != 0 && vtheta_samp != 0)
        {
          generateTrajectory( x, y, theta,
                              vx, vy, vtheta,
                              vx_samp, vy_samp, vtheta_samp,
                              acc_x, acc_y, acc_theta,
                              impossible_cost, *comp_traj);
          selectBestTrajectory(best_traj,comp_traj);
        }
      }
    }
  }*/

  //do we have a legal trajectory
  if (best_traj->cost_ >= 0)
  {
      // avoid oscillations of in place rotation and in place strafing
      setOscillationFlags(best_traj,x,y);
      possibleResetOscillationFlags(x,y);
  }
  else
  {
    //ROS_WARN("Couldn`t find local trajectory:%lf",best_traj->cost_);

  //ROS_INFO("Oscillation_flags: stuck_forward:%d,stuck_backward:%d,stuck_left_strafe:%d,stuck_right_strafe:%d,stuck_left:%d,stuck_right:%d",stuck_forward_move, stuck_backward_move,stuck_left_strafe, stuck_right_strafe,stuck_left, stuck_right);
    //resetOscillationFlags();
  }

  //ROS_INFO("Best trajectory :%lf,%lf,%lf,%lf",
  //                           best_traj->cost_,best_traj->xv_,
  //                           best_traj->yv_,best_traj->thetav_);
  return *best_traj;
}

void TrajectoryPlanner::selectBestTrajectory(base_local_planner::Trajectory* &best_traj,base_local_planner::Trajectory* &comp_traj)
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

bool TrajectoryPlanner::checkOscillationFlags(double vx, double vy, double vth)
{
  //ROS_INFO("Checking Oscillation flags:");
  //ROS_INFO("Oscillation_flags: stuck_forward:%d,stuck_backward:%d,stuck_left_strafe:%d,stuck_right_strafe:%d,stuck_left:%d,stuck_right:%d",stuck_forward_move, stuck_backward_move,stuck_left_strafe, stuck_right_strafe,stuck_left, stuck_right);
  //ROS_INFO("Vel:%lf,%lf,%lf",vx,vy,vth);
  if(stuck_forward_move && vx > 0.0) return true;
  if(stuck_backward_move && vx < 0.0) return true;
  if(stuck_left_strafe && vy > 0.0) return true;
  if(stuck_right_strafe && vy < 0.0) return true;
  if(stuck_left && vth > 0.0) return true;
  if(stuck_right && vth < 0.0) return true;
  return false;
}

void TrajectoryPlanner::setOscillationFlags(base_local_planner::Trajectory* traj,double x, double y)
{
  bool flag = false;
  //Check forward and backward oscillation
  if( traj->xv_ < 0.0 )
  {
    if(moving_forward)
    {
      stuck_forward_move = true;
      flag = true;
    }
    moving_backward = true;
    moving_forward = false; 
  }

  else if( traj->xv_ > 0.0 )
  {
    if(moving_backward)
    {
      stuck_backward_move = true;
      flag = true;
    }
    moving_backward = false;
    moving_forward = true; 
  }

  //Check oscillation of in place rotation and strafing
  if ( fabs(traj->xv_) <= min_vel_x_y_ ) 
  {
    if (traj->thetav_ < 0) 
    {
      if (rotating_left) 
      {
        stuck_left = true;
        flag = true;
      }
      rotating_left = false;
      rotating_right = true;
    } 
    
    if (traj->thetav_ > 0) 
    {
      if (rotating_right)
      {
        stuck_right = true;
        flag = true;
      }
      rotating_right = false;
      rotating_left = true;
    }
 
    if(traj->yv_ > 0) 
    {
      if (strafe_right) 
      {
        stuck_right_strafe = true;
        flag = true;
      }
      strafe_left = true;
      strafe_right = false;
    } 
    
    if(traj->yv_ < 0)
    {
      if (strafe_left) 
      {
        stuck_left_strafe = true;
        flag = true;
      }
      strafe_right = true;
      strafe_left = false;
    }
    if(flag)
    {
      //set the position we must move a certain distance away from
      prev_x_ = x;
      prev_y_ = y;
    }
  }
}

void TrajectoryPlanner::resetOscillationFlags()
{
    moving_forward = false;
    moving_backward = false;
    stuck_forward_move = false;
    stuck_backward_move = false;
    rotating_left = false;
    rotating_right = false;
    strafe_left = false;
    strafe_right = false;
    stuck_left = false;
    stuck_right = false;
    stuck_left_strafe = false;
    stuck_right_strafe = false;
}

void TrajectoryPlanner::possibleResetOscillationFlags(double x, double y)
{
  double dist = sqrt((x - prev_x_) * (x - prev_x_) + (y - prev_y_) * (y - prev_y_));
  //ROS_INFO("OSC_DIST:%lf",dist);
  if (dist > oscillation_reset_dist_) 
  {
    resetOscillationFlags();
  }
}

  //given the current state of the robot, find a good trajectory
  base_local_planner::Trajectory TrajectoryPlanner::findBestPath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel, 
      tf::Stamped<tf::Pose>& drive_velocities){

    Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
    Eigen::Vector3f vel(global_vel.getOrigin().getX(), global_vel.getOrigin().getY(), tf::getYaw(global_vel.getRotation()));

    //reset the map for new operations
    path_map_.resetPathDist();
    goal_map_.resetPathDist();

    //temporarily remove obstacles that are within the footprint of the robot
    std::vector<base_local_planner::Position2DInt> footprint_list =
        footprint_helper_.getFootprintCells(
            pos,
            footprint_spec_,
            costmap_,
            true);

    //mark cells within the initial footprint of the robot
    for (unsigned int i = 0; i < footprint_list.size(); ++i) 
    {
      path_map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
    }

    //make sure that we update our path based on the global plan and compute costs
    path_map_.setTargetCells(costmap_, global_plan_);
    goal_map_.setLocalGoal(costmap_, global_plan_);
    ROS_DEBUG("Path/Goal distance computed");

    //rollout trajectories and find the minimum cost one
    base_local_planner::Trajectory best = createTrajectories(pos[0], pos[1], pos[2],
        vel[0], vel[1], vel[2],
        acc_lim_x_, acc_lim_y_, acc_lim_theta_);
    ROS_DEBUG("Trajectories created");

    /*
    //If we want to print a ppm file to draw goal dist
    char buf[4096];
    sprintf(buf, "base_local_planner.ppm");
    FILE *fp = fopen(buf, "w");
    if(fp){
      fprintf(fp, "P3\n");
      fprintf(fp, "%d %d\n", map_.size_x_, map_.size_y_);
      fprintf(fp, "255\n");
      for(int j = map_.size_y_ - 1; j >= 0; --j){
        for(unsigned int i = 0; i < map_.size_x_; ++i){
          int g_dist = 255 - int(map_(i, j).goal_dist);
          int p_dist = 255 - int(map_(i, j).path_dist);
          if(g_dist < 0)
            g_dist = 0;
          if(p_dist < 0)
            p_dist = 0;
          fprintf(fp, "%d 0 %d ", g_dist, 0);
        }
        fprintf(fp, "\n");
      }
      fclose(fp);
    }
    */

    if(best.cost_ < 0){
      drive_velocities.setIdentity();
    }
    else{
      tf::Vector3 start(best.xv_, best.yv_, 0);
      drive_velocities.setOrigin(start);
      tf::Matrix3x3 matrix;
      matrix.setRotation(tf::createQuaternionFromYaw(best.thetav_));
      drive_velocities.setBasis(matrix);
    }

    return best;
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double TrajectoryPlanner::footprintCost(double x_i, double y_i, double theta_i){
    //check if the footprint is legal
    return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
  }


  void TrajectoryPlanner::getLocalGoal(double& x, double& y){
    x = path_map_.goal_x_;
    y = path_map_.goal_y_;
  }

};


