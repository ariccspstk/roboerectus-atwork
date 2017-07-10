/*********************************************************************:
* Modified by WangYiyan(Ian)based on base_local_planner trajecory_planner
*********************************************************************/

#include <aricc_base_local_planner/trajectory_planner.h>
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

namespace aricc_base_local_planner{

  void TrajectoryPlanner::reconfigureCB(AriccBaseLocalPlannerConfig &config,
    uint32_t level){
      
      if(setup_ && config.restore_defaults){
        config = default_config_;
        config.restore_defaults = false;
      }

      if(!setup_){
        default_config_ = config;
        setup_ = true;
      }
      boost::mutex::scoped_lock l(configuration_mutex_);

      max_vel_x_ = config.max_vel_x;
      min_vel_x_ = config.min_vel_x;
      
      max_vel_y_ = config.max_vel_y;
      min_vel_y_ = config.min_vel_y;

      min_vel_tran_ = config.min_trans_vel;
      max_vel_tran_ = config.max_trans_vel;
      
      max_vel_th_ = config.max_rot_vel;
      min_vel_th_ = -1.0*max_vel_th_;

      min_in_place_vel_th_ = config.min_in_place_rot_vel;

      sim_time_ = config.sim_time;
      sim_granularity_ = config.sim_granularity;
      angular_sim_granularity_ = config.angular_sim_granularity;

      pdist_scale_   = config.path_dist_bias;
      gdist_scale_   = config.goal_dist_bias;
      occdist_scale_ = config.occ_dist_scale;
      heading_scale_ = config.heading_bias;
      oscillation_reset_dist_ = config.oscillation_reset_dist;
      dist_rotate_to_goal_ = config.dist_rotate_to_goal;
      footprint_scale_threshold_ = config.footprint_scale_threshold;
      max_footprint_scale_factor_ = config.max_footprint_scale_factor; 

      int vx_samp = 0;
      int vy_samp = 0;
      int vth_samp = 0;
      vx_samp = config.vx_samples;
      vy_samp = config.vy_samples;
      vth_samp = config.vth_samples;
      if(vx_samp <= 0) {
        vx_samp = 1;
        config.vx_samples = vx_samp;
        ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
      }
      if(vy_samp <= 0) {
        vy_samp = 1;
        config.vy_samples = vy_samp;
        ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
      }
      if(vth_samp <= 0) {
        vth_samp = 1;
        config.vth_samples = vth_samp;
        ROS_WARN("You've specified that you don't want any samples in the theta dimension. We'll at least assume that you want to sample one value... so we're going to set vtheta_samples to 1 instead");
      }
    num_samples_[0] = vx_samp;
    num_samples_[1] = vy_samp;
    num_samples_[2] = vth_samp;
  }

  TrajectoryPlanner::TrajectoryPlanner(std::string name, 
    costmap_2d::Costmap2DROS* costmap_ros) :
    costmap_ros_(NULL), 
    world_model_(NULL),
    costmap_(NULL),
    dsrv_(ros::NodeHandle("~"+ name)),
    setup_(false){
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
  
    path_map_ = base_local_planner::MapGrid(
      costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    goal_map_ = base_local_planner::MapGrid(
      costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    
    ros::NodeHandle pnh("/"+ name);
    double acc_lim_x = 0.0;
    double acc_lim_y = 0.0;
    double acc_lim_th = 0.0;
    double dec_lim_x = 0.0;
    double dec_lim_y = 0.0;
    double dec_lim_th = 0.0;

    pnh.param("acc_lim_x",  acc_lim_x, 2.5);
    pnh.param("acc_lim_y",  acc_lim_y, 2.5);
    pnh.param("acc_lim_th", acc_lim_th, 2.5);
    pnh.param("dec_lim_x",  dec_lim_x, 2.5);
    pnh.param("dec_lim_y",  dec_lim_y, 2.5);
    pnh.param("dec_lim_th", dec_lim_th, 2.5);

    acc_lim_[0] = acc_lim_x;
    acc_lim_[1] = acc_lim_y;
    acc_lim_[2] = acc_lim_th;
    dec_lim_[0] = dec_lim_x;
    dec_lim_[1] = dec_lim_y;
    dec_lim_[2] = dec_lim_th;

    //Assuming this planner is being run within the navigation stack, we can
    //just do an upward search for the frequency at which its being run. This
    //also allows the frequency to be overwritten locally.
    std::string controller_frequency_param_name;
    if(!pnh.searchParam("controller_frequency", controller_frequency_param_name))
        sim_period_ = 0.05;
    else{
      double controller_frequency = 0;
      pnh.param(controller_frequency_param_name, controller_frequency, 20.0);
      if(controller_frequency > 0) sim_period_ = 1.0 / controller_frequency;
      else{
        ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        sim_period_ = 0.05;
      }
    }
    ROS_INFO("Sim period is set to %.2f", sim_period_);

    pnh.param("sim_time", sim_time_, 1.0);
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    
    dynamic_reconfigure::Server<AriccBaseLocalPlannerConfig>::CallbackType cb = boost::bind(&TrajectoryPlanner::reconfigureCB, this, _1, _2);
    dsrv_.setCallback(cb);

    map_viz_.initialize(name, costmap_ros_->getGlobalFrameID(), boost::bind(&TrajectoryPlanner::getCellCosts, this, _1, _2, _3, _4, _5, _6));
    //the robot is not stuck to begin with
    resetOscillationFlags();
    prev_pos_ = Eigen::Vector3f::Zero();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);

    ROS_WARN("Using AriccBaseLocalPlanner::Trajectory_planner and it is  initilized");
  }

  bool TrajectoryPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
    base_local_planner::MapCell cell = path_map_(cx, cy);
    base_local_planner::MapCell goal_cell = goal_map_(cx, cy);
    if (cell.within_robot) {
        return false;
    }
    occ_cost = costmap_->getCost(cx, cy);
    if (cell.target_dist == path_map_.obstacleCosts() ||
        cell.target_dist == path_map_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
    }
    path_cost = cell.target_dist;
    goal_cost = goal_cell.target_dist;
    double resolution = costmap_->getResolution();
    total_cost = pdist_scale_* resolution * path_cost + 
                 gdist_scale_ * resolution* goal_cost + 
                 occdist_scale_ * occ_cost;
    return true;
  }

  Eigen::Vector3f TrajectoryPlanner::computeNewPosition(
    const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, double dt){
    Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
    new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
    new_pos[1] = pos[1] + (vel[1] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
    new_pos[3] = pos[2] + vel[0] * dt;
    return new_pos;
  }

  void TrajectoryPlanner::selectBestTrajectory(
    base_local_planner::Trajectory* &best_traj,
    base_local_planner::Trajectory* &comp_traj){
    bool best_valid = best_traj->cost_ >= 0.0;
    bool comp_valid = comp_traj->cost_ >= 0.0;

    //If no valid traj to be compared, then return
    if(!comp_valid) return;

    base_local_planner::Trajectory* swap = NULL;
    if( comp_valid && 
        ( comp_traj->cost_ < best_traj->cost_ || !best_valid) ){
      swap = best_traj;
      best_traj = comp_traj;
      comp_traj = swap;
    }
  }

  /*
   *Checking the state of oscillation flags
   */
  bool TrajectoryPlanner::checkOscillationFlags(const Eigen::Vector3f& vel){
    //ROS_INFO("Checking Oscillation flags:");
    //ROS_INFO("Oscillation_flags: stuck_forward:%d,stuck_backward:%d,stuck_left_strafe:%d,stuck_right_strafe:%d,stuck_left:%d,stuck_right:%d",stuck_forward_move, stuck_backward_move,stuck_left_strafe, stuck_right_strafe,stuck_left, stuck_right);
    //ROS_INFO("Vel:%lf,%lf,%lf",vx,vy,vth);
    if(stuck_forward_      && vel[0] > 0.0) return true;
    if(stuck_backward_     && vel[0] < 0.0) return true;
    if(stuck_left_strafe_  && vel[1] > 0.0) return true;
    if(stuck_right_strafe_ && vel[1] < 0.0) return true;
    if(stuck_left_rotate_  && vel[2] > 0.0) return true;
    if(stuck_right_rotate_ && vel[2] < 0.0) return true;
    
    return false;
  }

  bool TrajectoryPlanner::setOscillationFlags(
    base_local_planner::Trajectory* traj){
    bool set_flag = false;
    //Check forward and backward oscillation
    if( traj->xv_ < 0.0 ){
      if(moving_forward_){
        stuck_forward_ = true;
        set_flag = true;
      }
      moving_backward_ = true;
      moving_forward_  = false; 
    }

    else if( traj->xv_ > 0.0 ){
      if(moving_backward_){
        stuck_backward_ = true;
        set_flag = true;
      }
      moving_forward_ = true; 
      moving_backward_ = false;
    }

    //Check oscillation of in place rotation and strafing
    if ( fabs(traj->xv_) <= min_vel_tran_ ){
      if (traj->thetav_ < 0){
        if (rotating_left_){
          stuck_left_rotate_ = true;
          set_flag = true;
        }
        rotating_right_ = true;
        rotating_left_ = false;
      }   
    
      if (traj->thetav_ > 0){
        if (rotating_right_){
          stuck_right_rotate_ = true;
          set_flag = true;
        }
        rotating_right_ = false;
        rotating_left_ = true;
      }
 
      if(traj->yv_ > 0){
        if (strafe_right_){
          stuck_right_strafe_ = true;
          set_flag = true;
        }
        strafe_left_ = true;
        strafe_right_ = false;
      } 
    
      if(traj->yv_ < 0){
        if (strafe_left_){
          stuck_left_strafe_ = true;
          set_flag = true;
        }
        strafe_right_ = true;
        strafe_left_ = false;
      }
    }
    return set_flag;
  }

  void TrajectoryPlanner::resetOscillationFlags(){
    moving_forward_  = false;
    moving_backward_ = false;
    rotating_left_   = false;
    rotating_right_  = false;
    strafe_left_     = false;
    strafe_right_    = false;

    stuck_forward_      = false;
    stuck_backward_     = false;
    stuck_left_rotate_  = false;
    stuck_right_rotate_ = false;
    stuck_left_strafe_  = false;
    stuck_right_strafe_ = false;
  }

  void TrajectoryPlanner::possibleResetOscillationFlags(
    const Eigen::Vector3f& pos, const Eigen::Vector3f& prev_pos){
    double dx = pos[0] - prev_pos[0];
    double dy = pos[1] - prev_pos[1];
    double sq_dist = dx*dx + dy*dy;
    //If robot has moved far enough, then reset oscillation flags
    if (sq_dist > oscillation_reset_dist_* oscillation_reset_dist_){
      resetOscillationFlags();
    }
  }

  /**
   * create and score a trajectory given the current pose of the robot and selected velocities
   */
  void TrajectoryPlanner::generateTrajectory(
    Eigen::Vector3f pos, 
    const Eigen::Vector3f& vel,
    base_local_planner::Trajectory& traj,
    double dist_to_goal){
    //any cell with a cost greater than the size of the map is impossible
    double impossible_cost = path_map_.obstacleCosts();
    //compute the magnitude of the velocities
    double vmag = sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
    double eps = 1e-4;

    if( ( (vmag + eps) < min_vel_tran_ &&
        ( fabs(vel[2])+eps) < min_rot_vel_ ) ||
        (vmag - eps) > max_vel_tran_ ||
        checkOscillationFlags(vel) ){
      //ROS_WARN("Got oscillation flags");
      traj.cost_ = -1.0;
      return;
    }

    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps = int(max((vmag * sim_time_) / sim_granularity_, fabs(vel[2]) / angular_sim_granularity_) + 0.5);
    if(num_steps == 0) traj.cost_ = -1.0;

    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vel[0]; 
    traj.yv_ = vel[1]; 
    traj.thetav_ = vel[2];
    traj.cost_ = -1.0;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;

    for(unsigned int i = 0; i < num_steps; ++i){
      //we don't want a path that goes off the know map
      unsigned int cell_x, cell_y;
      if(!costmap_->worldToMap(pos[0], pos[1], cell_x, cell_y)){
        //ROS_WARN("Path goes off the know map");
        traj.cost_ = -1.0;
        return;
      }

      //if footprint hits an obstacle this trajectory is invalid
      //Moveover, we need a certain velocity to scale the robot`s 
      //footprint to make it either slow down or stay further from 
      //obstacle
      double scale = 1.0;
      if(vmag > footprint_scale_threshold_){
        double ratio = (vmag - footprint_scale_threshold_) / 
                       (max_vel_tran_ - footprint_scale_threshold_);
        scale = max_footprint_scale_factor_ * ratio + 1.0;
      }
      double footprint_cost = footprintCost(pos, scale);
      if(footprint_cost < 0){
        //ROS_WARN("Robot hits obstacle");
        traj.cost_ = -1.0;
        return;
      }
      //Tring to get the maximum obstacle cost following the trajectory
      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_->getCost(cell_x, cell_y)));

      //update path and goal distances
      path_dist = path_map_(cell_x, cell_y).target_dist;
      goal_dist = goal_map_(cell_x, cell_y).target_dist;
      
      //if a point on this trajectory has no clear path to goal it is invalid
      //-2.0 means that the robot is blocked because of propagation failure
      if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
        //ROS_WARN("No clear path to goal");
        traj.cost_ = -2.0;
        return;
      }

      //the point is legal... add it to the trajectory
      traj.addPoint(pos[0], pos[1], pos[2]);

      //calculate new positions
      pos = computeNewPosition(pos, vel, dt);
      time += dt;
    } // end for i < numsteps
  
    //Calculate heading diffence between trajectory and global goal
    double goal_heading = tf::getYaw(global_plan_.back().pose.orientation);
    double goal_heading_diff = fabs( angles::shortest_angular_distance(pos[2],goal_heading));

    //ROS_INFO("Traj_heading_diff:%lf",traj_heading_diff*180.0/M_PI);
    double resolution = costmap_->getResolution();
    //If robot is heading to goal, then take into account of goal heading diff
    if(dist_to_goal < dist_rotate_to_goal_){
      traj.cost_ = pdist_scale_ * path_dist * resolution + 
                   gdist_scale_ * goal_dist * resolution + 
                   occdist_scale_ * occ_cost + 
                   heading_scale_ * goal_heading_diff;
      //ROS_INFO("Goal_heading_diff:%lf",goal_heading_diff*180.0/M_PI);
      //ROS_INFO("Cost:%lf",cost);
    }
    //If robot is far from goal, then take into account of trajectory heading diff
    else{
      traj.cost_ = pdist_scale_ * path_dist * resolution + 
                   gdist_scale_ * goal_dist * resolution + 
                   occdist_scale_ * occ_cost + 
                   0.0 * goal_heading_diff;
      //ROS_INFO("Traj_heading_diff:%lf",traj_heading_diff*180.0/M_PI);
      //ROS_INFO("Cost:%lf",cost);
    }
  }

  void TrajectoryPlanner::updatePlan(
    const vector<geometry_msgs::PoseStamped>& new_plan ){
    global_plan_.resize(new_plan.size());
    for(unsigned int i = 0; i < new_plan.size(); ++i){
      global_plan_[i] = new_plan[i];
    }
  }

  bool TrajectoryPlanner::checkTrajectory(const Eigen::Vector3f& pos, 
    const Eigen::Vector3f& vel){
    //ROS_INFO("Check trajectory");
    resetOscillationFlags();
    base_local_planner::Trajectory t; 

    generateTrajectory(pos, vel, t, 0.0);

    //if the trajectory is a legal one... the check passes
    if(t.cost_ >= 0) return true;

    //otherwise the check fails
    return false;
  }

  /*
   * create the trajectories we wish to score
   */
  base_local_planner::Trajectory TrajectoryPlanner::createTrajectories(
    const Eigen::Vector3f& pos, const Eigen::Vector3f& vel) {
  
    //Calculate the distance between robot and the last point of global plan
    double dist_to_goal =
         (pos[0] - global_plan_.back().pose.position.x) *
         (pos[0] - global_plan_.back().pose.position.x) + 
         (pos[1] - global_plan_.back().pose.position.y) * 
         (pos[1] - global_plan_.back().pose.position.y);
    dist_to_goal = sqrt(dist_to_goal);
  
    //compute feasible velocity limits in robot space
    Eigen::Vector3f max_vel  = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_vel  = Eigen::Vector3f::Zero();
    Eigen::Vector3f d_vel    = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();

    max_vel[0] = std::min(max_vel_x_,  vel[0] + acc_lim_[0] * sim_period_);
    max_vel[1] = std::min(max_vel_y_,  vel[1] + acc_lim_[1] * sim_period_);
    max_vel[2] = std::min(max_vel_th_, vel[2] + acc_lim_[2] * sim_period_);
  
    min_vel[0] = std::max(min_vel_x_,  vel[0] - dec_lim_[0] * sim_period_);
    min_vel[1] = std::max(min_vel_y_,  vel[1] - dec_lim_[1] * sim_period_);
    min_vel[2] = std::max(min_vel_th_, vel[2] - dec_lim_[2] * sim_period_);
      
    //we want to sample the velocity space regularly
    d_vel[0] = (max_vel[0] - min_vel[0]) / (std::max(1.0, double(num_samples_[0]) - 1));
    d_vel[1] = (max_vel[1] - min_vel[1]) / (std::max(1.0, double(num_samples_[1]) - 1));
    d_vel[2] = (max_vel[2] - min_vel[2]) / (std::max(1.0, double(num_samples_[2]) - 1));

    //keep track of the best trajectory seen so far
    base_local_planner::Trajectory* best_traj = &traj_one_;
    best_traj->cost_ = -1.0;

    base_local_planner::Trajectory* comp_traj = &traj_two_;
    comp_traj->cost_ = -1.0;

  
    vel_samp[0] = min_vel[0];
    //Loop through all x velocities
    for( unsigned int i = 0; i < num_samples_[0]; ++i){
      //loop through all y velocities
      vel_samp[1] = min_vel[1];
      for(unsigned int j = 0; j < num_samples_[1]; ++j){
        //loop through all theta velocities
        vel_samp[2] = min_vel[2];
        for(unsigned int k = 0; k < num_samples_[2]; ++k){
          generateTrajectory( pos, vel_samp, *comp_traj, dist_to_goal);
          //if the new trajectory is better... let's take it
          selectBestTrajectory(best_traj,comp_traj);
          vel_samp[2] += d_vel[2];
        }
        vel_samp[1] += d_vel[1];
      }
      vel_samp[0] += d_vel[0];
    }
  
    /*
    ROS_INFO("Oscillation_flags: stuck_forward:%d,stuck_backward:%d,stuck_left_strafe:%d,stuck_right_strafe:%d,stuck_left:%d,stuck_right:%d",stuck_forward_move, stuck_backward_move,stuck_left_strafe, stuck_right_strafe,stuck_left, stuck_right);*/
 
    //do we have a legal trajectory
    if (best_traj->cost_ >= 0) {
      // avoid oscillations of in place rotation and in place strafing
      if( setOscillationFlags(best_traj) ) prev_pos_ = pos;

      //If has oscillation flag on, check and try to reset it
      if( stuck_forward_      || stuck_backward_     ||
          stuck_left_rotate_  || stuck_right_rotate_ ||
          stuck_left_strafe_  ||stuck_right_strafe_ )
        possibleResetOscillationFlags(pos, prev_pos_);
    }
    else{
      ROS_WARN("Couldn`t find trajectory:%lf",best_traj->cost_);
    
    //ROS_INFO("Oscillation_flags: stuck_forward:%d,stuck_backward:%d,stuck_left_strafe:%d,stuck_right_strafe:%d,stuck_left:%d,stuck_right:%d",stuck_forward_move, stuck_backward_move,stuck_left_strafe, stuck_right_strafe,stuck_left, stuck_right);
    //resetOscillationFlags();
    }  
  
    //ROS_INFO("Best trajectory :%lf,%lf,%lf,%lf",
    //                           best_traj->cost_,best_traj->xv_,
    //                           best_traj->yv_,best_traj->thetav_);
    return *best_traj;
  }

  //given the current state of the robot, find a good trajectory
  base_local_planner::Trajectory TrajectoryPlanner::findBestPath(
    tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel, 
    tf::Stamped<tf::Pose>& drive_velocities){

    // make sure the configuration doesn't change mid run
    boost::mutex::scoped_lock l(configuration_mutex_);

    Eigen::Vector3f pos(global_pose.getOrigin().getX(), 
      global_pose.getOrigin().getY(), 
      tf::getYaw(global_pose.getRotation()));
    Eigen::Vector3f vel(
      global_vel.getOrigin().getX(), 
      global_vel.getOrigin().getY(), 
      tf::getYaw(global_vel.getRotation()));

    //Make sure to get the latest costmap to calculate trajectory
    costmap_ = costmap_ros_->getCostmap();

    //reset the map for new operations
    path_map_.resetPathDist();
    goal_map_.resetPathDist();

    //temporarily remove obstacles that are within the footprint of the robot
    std::vector<base_local_planner::Position2DInt> footprint_list =
        footprint_helper_.getFootprintCells(
            pos,
            footprint_spec_,
            *costmap_,
            true);

    //mark cells within the initial footprint of the robot
    for (unsigned int i = 0; i < footprint_list.size(); ++i){
      path_map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
    }

    //make sure that we update our path based on the global plan and compute costs
    path_map_.setTargetCells(*costmap_, global_plan_);
    goal_map_.setLocalGoal(*costmap_, global_plan_);
    ROS_DEBUG("Path/Goal distance computed");

    //rollout trajectories and find the minimum cost one
    base_local_planner::Trajectory best = createTrajectories(pos, vel);
    ROS_DEBUG("Trajectories created");

    //If no legal trajectory, stop the robot
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
    //Publishing costmap for visualization 
    map_viz_.publishCostCloud(costmap_);
    return best;
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double TrajectoryPlanner::footprintCost(
    const Eigen::Vector3f& pos, double scale){
    //check if the footprint is legal
    double cos_th = cos(pos[2]);
    double sin_th = sin(pos[2]);

    std::vector<geometry_msgs::Point> oriented_footprint;
    for(unsigned int i = 0; i < footprint_spec_.size(); ++i){
      geometry_msgs::Point new_pt;
      new_pt.x = pos[0] + scale*(footprint_spec_[i].x*cos_th - 
        footprint_spec_[i].y*sin_th);
      new_pt.y = pos[0] + scale*(footprint_spec_[i].y*sin_th + 
        footprint_spec_[i].y*cos_th);
      oriented_footprint.push_back(new_pt);
    }
    geometry_msgs::Point robot_pos;
    robot_pos.x = pos[0];
    robot_pos.y = pos[1];

    //To check the legality of robot in the world coordinate
    return world_model_->footprintCost(robot_pos, oriented_footprint, inscribed_radius_, circumscribed_radius_);
  }
};


