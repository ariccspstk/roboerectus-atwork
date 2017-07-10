/*********************************************************************
* Modified by WangYiyan(Ian)based on base_local_planner trajecory_planner
*********************************************************************/

#ifndef TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_H_
#define TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_H_

#include <vector>
#include <cmath>
#include <Eigen/Core>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>

#include <base_local_planner/footprint_helper.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/Position2DInt.h>
#include <base_local_planner/map_cell.h>
#include <base_local_planner/map_grid.h>
#include <base_local_planner/map_grid_visualizer.h>

//For configuration
#include <aricc_base_local_planner/AriccBaseLocalPlannerConfig.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>

//for some datatypes
#include <tf/transform_datatypes.h>

//for creating a local cost grid

namespace aricc_base_local_planner {
  /**
   * @class TrajectoryPlanner
   * @brief Computes control velocities for a robot given a costmap, a plan, and the robot's position in the world. 
   */
  class TrajectoryPlanner{
    public:
      /**
       * @brief  Constructs a trajectory controller
       * @param name The name of planner 
       * @param costmap_ros The pointer of costmap2DROS 
       */
      TrajectoryPlanner( std::string name, 
        costmap_2d::Costmap2DROS* costmap_ros );

      /**
       * @brief  Destructs a trajectory controller
       */
      ~TrajectoryPlanner(){delete world_model_;};


      /**
       * @brief  Given the current position, orientation, and velocity of the robot, return a trajectory to follow
       * @param global_pose The current pose of the robot in world space 
       * @param global_vel The current velocity of the robot in world space
       * @param drive_velocities Will be set to velocities to send to the robot base
       * @return The selected path or trajectory
       */
      base_local_planner::Trajectory findBestPath(
        tf::Stamped<tf::Pose> global_pose, 
        tf::Stamped<tf::Pose> global_vel,
        tf::Stamped<tf::Pose>& drive_velocities);

      /**
       * @brief  Update the plan that the controller is following
       * @param new_plan A new plan for the controller to follow 
       */
      void updatePlan(
        const std::vector<geometry_msgs::PoseStamped>& new_plan);

      /**
       * @brief  Accessor for the goal the robot is currently pursuing in world corrdinates
       * @param x Will be set to the x position of the local goal 
       * @param y Will be set to the y position of the local goal 
       */
      void getLocalGoal(double& x, double& y);

      /**
       * @brief  Generate and score a single trajectory
       * @param pos The position of the robot  
       * @param vel The velocity of the robot  
       * @return True if the trajectory is legal, false otherwise
       */
      bool checkTrajectory(const Eigen::Vector3f& pos,
        const Eigen::Vector3f& vel);


      /**
       * @brief Compute the components and total cost for a map grid cell
       * @param cx The x coordinate of the cell in the map grid
       * @param cy The y coordinate of the cell in the map grid
       * @param path_cost Will be set to the path distance component of the cost function
       * @param goal_cost Will be set to the goal distance component of the cost function
       * @param occ_cost Will be set to the costmap value of the cell
       * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
       * @return True if the cell is traversible and therefore a legal location for the robot to move to
       */
      bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

      /** @brief Set the footprint specification of the robot. */
      void setFootprint( std::vector<geometry_msgs::Point> footprint )      { footprint_spec_ = footprint; }

      /** @brief Return the footprint specification of the robot. */
      geometry_msgs::Polygon getFootprintPolygon() const { return costmap_2d::toPolygon(footprint_spec_); }
      std::vector<geometry_msgs::Point> getFootprint() const { return footprint_spec_; }

      /** @brief Get deceleration limit of the robot. */
      inline Eigen::Vector3f getAccLimits(){ return acc_lim_;};

      /** @brief Get deceleration limit of the robot. */
      inline Eigen::Vector3f getDecLimits(){ return dec_lim_;};

      /** @brief Get sim_period parameter. */
      inline double getSimPeriod(){ return sim_period_;};

    private:
      /**
       * @brief Reconfigures the trajectory planner
       */
      void reconfigureCB(AriccBaseLocalPlannerConfig &cfg,
        uint32_t level);

      /**
       * @brief  Create the trajectories we wish to explore, score them, and return the best option
       * @param pos The position of the robot(x,y,theta)  
       * @param vel The velocity of the robot(x,y,theta)
       * @return 
       */
      base_local_planner::Trajectory createTrajectories(
        const Eigen::Vector3f& pos, const Eigen::Vector3f& vel);

      /**
       * @brief  Generate a single trajectory
       * @param pos The position of the robot  
       * @param vel The velocity of the robot  
       * @param traj The pointer of trajectory  
       * @param dist_to_goal The distance between current robot pose and the final goal.  
       */
      void generateTrajectory(
          Eigen::Vector3f pos, 
          const Eigen::Vector3f& vel,
          base_local_planner::Trajectory& traj,
          double dist_to_goal);

      /**
       * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       * @param pos The position of the robot 
       * @param scale The scale factor of robot footprint 
       * @return 
       */
      double footprintCost(const Eigen::Vector3f& pos, double scale);
      
       /**
       * @brief Set oscillation flags
       * @param t The generated trajectory 
       * @return void
       */
      bool setOscillationFlags(base_local_planner::Trajectory* t);
      
      /**
       * @brief  Reset Oscillation flags
       * @return 
       */
      void resetOscillationFlags();
      
      /**
       * @brief  Checks whether we can reset oscillation flags
       * @param pos The current position of the robot 
       * @param prev_pos The previous position of the robot 
       * @return 
       */
      void possibleResetOscillationFlags(const Eigen::Vector3f& pos, 
        const Eigen::Vector3f& prev_pos);
      
      /**
       * @brief  Checks whether the oscillation flags is set
       * @param vel The velocity of the robot 
       * @return 
       */
      bool checkOscillationFlags(const Eigen::Vector3f& vel);
      
      /**
       * @brief Compare to the generated trajectory and select the best one
       * @param best_traj The best trajectory
       * @param comp_traj The generated trajectory 
       * @return 
       */
      void selectBestTrajectory(
        base_local_planner::Trajectory* &best_traj,
        base_local_planner::Trajectory* &comp_traj );
      
      /**
       * @brief  Compute new robot position based on velocity
       * @param  pos The current position of robot
       * @param  vel The current velocity of robot
       * @param  dt The timestep to take
       * @return The new x position 
       */
      Eigen::Vector3f computeNewPosition(
        const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, 
        double dt);

      bool setup_;
      aricc_base_local_planner::AriccBaseLocalPlannerConfig default_config_;
      base_local_planner::FootprintHelper footprint_helper_;
      base_local_planner::MapGrid path_map_; 
      base_local_planner::MapGrid goal_map_; 
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_; ///< @brief Provides access to cost map information
      base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller uses for collision detection
      std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief The footprint specification of the robot
      std::vector<geometry_msgs::PoseStamped> global_plan_; ///< @brief The global path for the robot to follow
     
      //Oscillation flags
      bool stuck_forward_, stuck_backward_;
      bool moving_forward_, moving_backward_;
      bool stuck_left_rotate_, stuck_right_rotate_; 
      bool rotating_left_, rotating_right_;
      bool stuck_left_strafe_, stuck_right_strafe_;
      bool strafe_right_, strafe_left_;
      
      double sim_time_;
      double sim_granularity_;
      double angular_sim_granularity_;

      Eigen::Vector3f acc_lim_;
      Eigen::Vector3f dec_lim_;
      Eigen::Vector3f prev_pos_; 
      Eigen::Vector3f num_samples_;  

      double pdist_scale_;
      double gdist_scale_;
      double occdist_scale_;
      double heading_scale_;
      double oscillation_reset_dist_;

      base_local_planner::Trajectory traj_one_;
      base_local_planner::Trajectory traj_two_;
      //Velocity limits for the controller
      double max_vel_x_;
      double min_vel_x_;
      double max_vel_y_;
      double min_vel_y_;
      double max_vel_th_;
      double min_vel_th_;
      double min_in_place_vel_th_; 

      double min_vel_tran_;
      double max_vel_tran_;
      double min_rot_vel_;
      double dist_rotate_to_goal_;

      double sim_period_;

      double inscribed_radius_, circumscribed_radius_;
      double footprint_scale_threshold_;
      double max_footprint_scale_factor_;
     
      boost::mutex configuration_mutex_;
      dynamic_reconfigure::Server<AriccBaseLocalPlannerConfig> dsrv_;
      base_local_planner::MapGridVisualizer map_viz_;
  };
};

#endif
