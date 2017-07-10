/*********************************************************************
* Author: Eitan Marder-Eppstein
  Modified by Ian WangYiyan
*********************************************************************/
#ifndef TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_
#define TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <string>
#include <angles/angles.h>
#include <nav_core/base_local_planner.h>
#include <aricc_base_local_planner/trajectory_planner.h>
#include <base_local_planner/odometry_helper_ros.h>

namespace aricc_base_local_planner {
  /**
   * @class TrajectoryPlannerROS
   * @brief A ROS wrapper for the trajectory controller that queries the param server to construct a controller
   */
  class TrajectoryPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Default constructor for the ros wrapper
       */
      TrajectoryPlannerROS();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      TrajectoryPlannerROS(std::string name,
                           tf::TransformListener* tf,
                           costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~TrajectoryPlannerROS();
      
      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

      bool isInitialized() {
        return initialized_;
      }

    private:
      /**
       * @brief Once a goal position is reached... rotate to the goal orientation
       * @param  global_pose The pose of the robot in the global frame
       * @param  robot_vel The velocity of the robot
       * @param  goal_th The desired th value for the goal
       * @param  cmd_vel The velocity commands to be filled
       * @return  True if a valid trajectory was found, false otherwise
       */
      bool rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, 
        const tf::Stamped<tf::Pose>& robot_vel, double goal_th, 
        geometry_msgs::Twist& cmd_vel);

      /**
       * @brief Stop the robot taking into account acceleration limits
       * @param  global_pose The pose of the robot in the global frame
       * @param  robot_vel The velocity of the robot
       * @param  cmd_vel The velocity commands to be filled
       * @return  True if a valid trajectory was found, false otherwise
       */
      bool stopWithAccLimits(const tf::Stamped<tf::Pose>& global_pose,
         const tf::Stamped<tf::Pose>& robot_vel, 
         geometry_msgs::Twist& cmd_vel);

      inline double sign(double x){
        return x < 0.0 ? -1.0 : 1.0;
      }

      boost::shared_ptr<TrajectoryPlanner> tc_; ///< @brief The trajectory controller

      costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use
      costmap_2d::Costmap2D *costmap_;
      tf::TransformListener* tf_; ///< @brief Used for transforming point clouds
      double rot_stopped_velocity_;
      double trans_stopped_velocity_;
      double xy_goal_tolerance_; 
      double yaw_goal_tolerance_; 
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      bool prune_plan_;
      double max_vel_th_; 
      double min_vel_th_;
      double min_in_place_vel_th_;
      bool rotating_to_goal_;
      bool latch_xy_goal_tolerance_; 
      double xy_tolerance_latch_;
      ros::Publisher g_plan_pub_, l_plan_pub_;

      bool initialized_;
      base_local_planner::OdometryHelperRos odom_helper_;
  };
};
#endif
