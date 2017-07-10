#ifndef YOUBOTTELEOP_H_
#define YOUBOTTELEOP_H_

#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>


#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include "brics_actuator/JointPositions.h"
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

class YoubotTeleop{
  public:
	YoubotTeleop();
	~YoubotTeleop();
	void spin();
  private:
	ros::NodeHandle nh_;
	ros::Publisher pub_vel_;
	ros::Publisher pub_arm_joints_;
	ros::Publisher pub_gripper_joints_;
	ros::Subscriber sub_joy_;
        std::vector< std::vector<double> > arm_pose_list_;
        unsigned int btnCounter_;
        unsigned int btnNum_;
        unsigned int rate_;

	// params
	int axis_linear_x_;
	int axis_linear_y_;
	int axis_angular_;
	double scale_linear_;
	double scale_angular_;

	void readParams();
        void loadArmPoses();
	void joyCallback( const sensor_msgs::JoyConstPtr& msg);
        void moveArm(std::vector<double> joints);
};


#endif /* JOYSTICKTELEOP_H_ */
