/*********************************************************************
* Author: Eitan Marder-Eppstein
* Modified: Ian Wang 
  based on ROS base local planner trajectory_planner
*********************************************************************/

#include <aricc_base_local_planner/trajectory_planner_ros.h>
#include <sys/time.h>
#include <boost/tokenizer.hpp>
#include <Eigen/Core>
#include <cmath>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

PLUGINLIB_EXPORT_CLASS(aricc_base_local_planner::TrajectoryPlannerROS, nav_core::BaseLocalPlanner)

namespace aricc_base_local_planner {
  TrajectoryPlannerROS::TrajectoryPlannerROS(
    std::string name, tf::TransformListener* tf, 
    costmap_2d::Costmap2DROS* costmap_ros) :
    costmap_ros_(NULL), tf_(NULL), 
    initialized_(false), odom_helper_("odom") {

    //initialize the planner
    initialize(name, tf, costmap_ros);
  }

  void TrajectoryPlannerROS::initialize( 
    std::string name,
    tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* costmap_ros){
    if(! isInitialized()) {
      ros::NodeHandle pnh("~/" + name);
      g_plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = pnh.advertise<nav_msgs::Path>("local_plan", 1);

      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      rotating_to_goal_ = false;
    
      pnh.param("max_rotate_vel", max_vel_th_, 1.0);
      min_vel_th_ = -1.0 * max_vel_th_;
      pnh.param("min_in_place_rotate_vel", min_in_place_vel_th_, 0.1);

      pnh.param("prune_plan", prune_plan_, true);
      pnh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
      pnh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
      pnh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);
      pnh.param("rot_stopped_velocity", rot_stopped_velocity_, 1e-2);
      pnh.param("trans_stopped_velocity", trans_stopped_velocity_, 1e-2);
    
      tc_ = boost::shared_ptr<TrajectoryPlanner>
        (new TrajectoryPlanner( name, costmap_ros_));
      ROS_WARN("Using aricc base local planner.");
      initialized_ = true;
    } 
    else{
      ROS_WARN("AriccBasePlanner/TrajectoryPlannerROS: This planner has already been initialized, doing nothing");
    }
  }

  TrajectoryPlannerROS::~TrajectoryPlannerROS() {
    //make sure to clean things up
  }

  bool TrajectoryPlannerROS::stopWithAccLimits(
    const tf::Stamped<tf::Pose>& global_pose, 
    const tf::Stamped<tf::Pose>& robot_vel, 
    geometry_msgs::Twist& cmd_vel){
    //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
    //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
    Eigen::Vector3f dec_lim = tc_->getDecLimits();
    double sim_period = tc_->getSimPeriod();
    double vx = sign(robot_vel.getOrigin().x()) * std::max(0.0, (fabs(robot_vel.getOrigin().x()) - dec_lim[0] * sim_period));
    double vy = sign(robot_vel.getOrigin().y()) * std::max(0.0, (fabs(robot_vel.getOrigin().y()) - dec_lim[1] * sim_period));

    double vel_yaw = tf::getYaw(robot_vel.getRotation());
    double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - dec_lim[2] * sim_period));

    //we do want to check whether or not the command is valid
    double yaw = tf::getYaw(global_pose.getRotation());
    bool valid_cmd = tc_->checkTrajectory( 
      Eigen::Vector3f(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw), 
      Eigen::Vector3f(vx, vy, vth) );

    //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
    if(valid_cmd){
      ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
      cmd_vel.linear.x = vx;
      cmd_vel.linear.y = vy;
      cmd_vel.angular.z = vth;
      return true;
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  bool TrajectoryPlannerROS::rotateToGoal(
    const tf::Stamped<tf::Pose>& global_pose, 
    const tf::Stamped<tf::Pose>& robot_vel, 
    double goal_th, geometry_msgs::Twist& cmd_vel){
    Eigen::Vector3f acc_lim = tc_->getAccLimits();
    Eigen::Vector3f dec_lim = tc_->getDecLimits();
    double sim_period = tc_->getSimPeriod();

    double yaw = tf::getYaw(global_pose.getRotation());
    double vel_yaw = tf::getYaw(robot_vel.getRotation());
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

    double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_th_,
        std::max(min_in_place_vel_th_, ang_diff)) : std::max(min_vel_th_,
        std::min(-1.0 * min_in_place_vel_th_, ang_diff));

    //take the acceleration limits of the robot into account
    double max_acc_vel = fabs(vel_yaw) + acc_lim[2] * sim_period;
    double min_acc_vel = fabs(vel_yaw) - dec_lim[2] * sim_period;

    v_theta_samp = sign(v_theta_samp) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

    //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
    double max_speed_to_stop = sqrt(2 * dec_lim[2] * fabs(ang_diff)); 
    v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop, fabs(v_theta_samp));

    // Re-enforce min_in_place_vel_th_.  It is more important than the acceleration limits.
    v_theta_samp = v_theta_samp > 0.0
      ? std::min( max_vel_th_, std::max( min_in_place_vel_th_, v_theta_samp ))
      : std::max( min_vel_th_, std::min( -1.0 * min_in_place_vel_th_, v_theta_samp ));

    //we still want to lay down the footprint of the robot and check if the action is legal
    bool valid_cmd = tc_->checkTrajectory(
      Eigen::Vector3f(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw), Eigen::Vector3f(0.0, 0.0, v_theta_samp));

    ROS_DEBUG("Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);

    if(valid_cmd){
      cmd_vel.angular.z = v_theta_samp;
      return true;
    }

    cmd_vel.angular.z = 0.0;
    return false;

  }

  bool TrajectoryPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;

    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    xy_tolerance_latch_ = false;

    return true;
  }

  bool TrajectoryPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> local_plan;
    tf::Stamped<tf::Pose> global_pose;
    if (!costmap_ros_->getRobotPose(global_pose)) {
      ROS_ERROR("Could not get robot pose in costmap!");
      return false;
    }


    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    //get the global plan in our frame
    if (!base_local_planner::transformGlobalPlan(*tf_, global_plan_, 
      global_pose, *costmap_, costmap_ros_->getGlobalFrameID(), 
      transformed_plan)) {
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    //now we'll prune the plan based on the position of the robot
    if(prune_plan_)
      base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);
    
    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = costmap_ros_->getBaseFrameID();

    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty())
      return false;

    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.getOrigin().getX();
    double goal_y = goal_point.getOrigin().getY();
    double yaw = tf::getYaw(goal_point.getRotation());
    double goal_th = yaw;

    //check to see if we've reached the goal position
    if (xy_tolerance_latch_ || (base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)) {
      //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll just rotate in place
      if (latch_xy_goal_tolerance_) {
        xy_tolerance_latch_ = true;
      }

      double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
      //check to see if the goal orientation has been reached
      if (fabs(angle) <= yaw_goal_tolerance_) {
        //set the velocity command to zero
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
      } 
      else {
        //we need to call the next two lines to make sure that the trajectory
        //planner updates its path distance and goal distance grids
        tc_->updatePlan(transformed_plan);
        base_local_planner::Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);

        //copy over the odometry information
        nav_msgs::Odometry base_odom;
        odom_helper_.getOdom(base_odom);

        //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
        if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)) {
          if ( ! stopWithAccLimits(global_pose, robot_vel, cmd_vel)) {
            return false;
          }
        }
        //if we're stopped... then we want to rotate to goal
        else{
          //set this so that we know its OK to be moving
          rotating_to_goal_ = true;
          if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel)) {
            return false;
          }
        }
      }

      //publish an empty plan because we've reached our goal position
      base_local_planner::publishPlan(transformed_plan, g_plan_pub_);
      base_local_planner::publishPlan(local_plan, l_plan_pub_);

      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }//end of if condition of reaching goal

    tc_->updatePlan(transformed_plan);

    //compute what trajectory to drive along
    base_local_planner::Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

    //if we cannot move... tell someone
    if (path.cost_ < 0) {
      ROS_DEBUG_NAMED("trajectory_planner_ros",
          "The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
      local_plan.clear();
      base_local_planner::publishPlan(transformed_plan, g_plan_pub_);
      base_local_planner::publishPlan(local_plan, l_plan_pub_);
      return false;
    }

    ROS_DEBUG_NAMED("trajectory_planner_ros", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for (unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);
      tf::Stamped<tf::Pose> p =
          tf::Stamped<tf::Pose>(tf::Pose(
              tf::createQuaternionFromYaw(p_th),
              tf::Point(p_x, p_y, 0.0)),
              ros::Time::now(),
              costmap_ros_->getGlobalFrameID());
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer
    base_local_planner::publishPlan(transformed_plan, g_plan_pub_);
    base_local_planner::publishPlan(local_plan, l_plan_pub_);
    return true;
  }

  bool TrajectoryPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    odom_helper_.getOdom(base_odom);
    tf::Stamped<tf::Pose> global_pose;
    costmap_ros_->getRobotPose(global_pose);
    return base_local_planner::isGoalReached(
        *tf_,
        global_plan_,
        *costmap_,
        costmap_ros_->getGlobalFrameID(),
        global_pose,
        base_odom,
        rot_stopped_velocity_, 
        trans_stopped_velocity_,
        xy_goal_tolerance_, 
        yaw_goal_tolerance_);
  }

};
