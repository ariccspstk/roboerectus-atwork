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
#ifndef LOCAL_MOVE_RECOVERY_H_
#define LOCAL_MOVE_RECOVERY_H_
#include <ros/ros.h>

#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/footprint_helper.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/trajectory.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>

namespace local_move_recovery{
  /**
   * @class LocalMoveRecovery
   * @brief A recovery behavior that rotates the robot in-place to attempt to clear out space
   */
  class LocalMoveRecovery : public nav_core::RecoveryBehavior {
    public:
      /**
       * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
       * @param  
       * @return 
       */
      LocalMoveRecovery();

      /**
       * @brief  Initialization function for the RotateRecovery recovery behavior
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack 
       * @param local_costmap A pointer to the local_costmap used by the navigation stack 
       */
      void initialize(std::string name, tf::TransformListener* tf, 
          costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      /**
       * @brief  Run the RotateRecovery recovery behavior.
       */
      void runBehavior();

      /**
       * @brief  Destructor for the rotate recovery behavior
       */
      ~LocalMoveRecovery();

    private:
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      costmap_2d::Costmap2D* costmap_;
      std::string name_;
      tf::TransformListener* tf_;
      bool initialized_; 
      double acc_lim_x_;
      double acc_lim_y_;
      double acc_lim_th_; 
      double max_vel_x_;
      double min_vel_x_;
      double max_vel_y_;
      double min_vel_y_;
      double max_vel_th_;
      double min_vel_th_;
      double min_in_place_vel_th_;
      double max_trans_vel_;
      double min_trans_vel_;
      double max_rot_vel_;
      double min_rot_vel_;
      double sim_time_;
      double angular_sim_granularity_;
      double sim_granularity_;
      double sim_period_;
      int vx_samples_;
      int vy_samples_;
      int vtheta_samples_;
      double target_dist_, current_dist_;
      double tolerance_; 
      double frequency_;
      double occdist_scale_;
      std::vector<geometry_msgs::Point> footprint_spec_;
      double inscribed_radius_, circumscribed_radius_;
      
      double reset_distance_;
      std::set<std::string> clearable_layers_; ///< Layer names which will be cleared.

      base_local_planner::CostmapModel* world_model_;
      base_local_planner::Trajectory traj_one_, traj_two_;
      ros::Publisher vel_pub_;
      base_local_planner::OdometryHelperRos odom_helper_;
     
      bool isReachedGoal(double current_dist, double tolerance);
      base_local_planner::Trajectory createTrajectories(double x, double y, double theta,
                               double vx, double vy, double vtheta, 
                               double acc_x, double acc_y, double acc_theta);

      void generateTrajectory(
        double x, double y, double theta,
        double vx, double vy, double vtheta,
        double vx_samp, double vy_samp, double vtheta_samp,
        double acc_x, double acc_y, double acc_theta,
        base_local_planner::Trajectory& traj);

      void selectBestTrajectory(
        base_local_planner::Trajectory* &best_traj,
        base_local_planner::Trajectory* &comp_traj);

      void publishVel(double vx, double vy, double vth);

      double footprintCost(double x_i, double y_i, double theta_i);

      inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt){
         return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
       }

      inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt){
        return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
      }

      inline double computeNewThetaPosition(double thetai, double vth, double dt){
         return thetai + vth * dt;
       }

      inline double computeNewVelocity(double vg, double vi, double a_max, double dt){
        if((vg - vi) >= 0) {
          return std::min(vg, vi + a_max * dt);
        }
        return std::max(vg, vi - a_max * dt);
       }
 

  };
};
#endif  
