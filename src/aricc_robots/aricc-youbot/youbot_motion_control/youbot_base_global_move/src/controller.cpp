#include <actionlib/client/simple_action_client.h>
#include "dynamic_reconfigure/server.h"
#include <youbot_arm_joints/JointAction.h>
#include <youbot_arm_joints/ControllerConfig.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <assert.h>
#include "ros/ros.h"

class YoubotArmJointsController{
  public:
    typedef actionlib::SimpleActionClient<youbot_arm_joints::JointAction> ActionClient;
    typedef youbot_arm_joints::ControllerConfig Config;

    YoubotArmJointsController():nh_("~"),rate_(5){
      client_.reset( new ActionClient("youbot_arm_joints_server", true) );
      if(!client_->waitForServer(ros::Duration(5.0))){
        ROS_ERROR("youbot_arm_joints_server is not excuting");
      }
      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (nh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&YoubotArmJointsController::configCallback, this, _1, _2);
      srv_->setCallback(f);
    }

    ~YoubotArmJointsController(){
    }

    void spin(){
      ros::Rate r(rate_);
      while( nh_.ok() ){
        ros::spinOnce();
        r.sleep();
      }
    }

  private:
    ros::NodeHandle nh_;
    unsigned int rate_;
    boost::shared_ptr<ActionClient> client_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    //boost::mutex mutex_;
    std::vector<double> joints_;
    uint16_t timeout_;

    void configCallback(Config &config, uint32_t level){
      //boost::mutex::scoped_lock lock(mutex_);
      joints_.clear();
      if(config.straight){
        joints_.push_back(2.93);
        joints_.push_back(1.31);
        joints_.push_back(-2.51);
        joints_.push_back(1.73);
        joints_.push_back(2.88);
        
        config.joint0 = joints_[0];
        config.joint1 = joints_[1];
        config.joint2 = joints_[2];
        config.joint3 = joints_[3];
        config.joint4 = joints_[4];
        config.initial = false;
      }
      else if(config.initial){
        joints_.push_back(0.025);
        joints_.push_back(0.025);
        joints_.push_back(-0.025);
        joints_.push_back(0.025);
        joints_.push_back(0.115);
        
        config.joint0 = joints_[0];
        config.joint1 = joints_[1];
        config.joint2 = joints_[2];
        config.joint3 = joints_[3];
        config.joint4 = joints_[4];
        config.straight = false;
      }
      else if(!config.straight && !config.initial){
        joints_.push_back(config.joint0);
        joints_.push_back(config.joint1);
        joints_.push_back(config.joint2);
        joints_.push_back(config.joint3);
        joints_.push_back(config.joint4);
        //joints_.push_back(config.gripper_l);
        //joints_.push_back(config.gripper_r);
      }
      timeout_ = static_cast<uint16_t>(config.timeout);
      moveArm();
    }
    
    void moveArm(){
      youbot_arm_joints::JointGoal goal;
      goal.timeout = timeout_;
      goal.name = "arm";
      goal.joints = joints_;
      client_->sendGoal(goal);
    }
};

int main( int argc, char** argv ){
  ros::init( argc, argv, "youbot_arm_joints_controller" );
  YoubotArmJointsController rt_;
  rt_.spin();
  return 0;
}
