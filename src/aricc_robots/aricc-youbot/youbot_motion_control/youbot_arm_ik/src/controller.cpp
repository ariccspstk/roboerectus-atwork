#include <actionlib/client/simple_action_client.h>
#include "dynamic_reconfigure/server.h"
#include <youbot_arm_ik/IkAction.h>
#include <youbot_arm_ik/ControllerConfig.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <assert.h>
#include "ros/ros.h"

class YoubotArmIkController{
  public:
    typedef actionlib::SimpleActionClient<youbot_arm_ik::IkAction> ActionClient;
    typedef youbot_arm_ik::ControllerConfig Config;

    YoubotArmIkController():nh_("~"),rate_(5){
      client_.reset( new ActionClient("youbot_arm_ik_server", true) );
      if(!client_->waitForServer(ros::Duration(5.0))){
        ROS_ERROR("youbot_arm_ik_server is not excuting");
      }
      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (nh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&YoubotArmIkController::configCallback, this, _1, _2);
      srv_->setCallback(f);
    }

    ~YoubotArmIkController(){
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
    geometry_msgs::Point pos_;
    geometry_msgs::Vector3 ori_;
    double timeout_;
    double duration_;
    int method_;
    double pitch_;
    int id_;
    bool arm_to_front_;
    bool arm_bended_up_;
    bool gripper_downwards_;
    bool move_arm_;

    void configCallback(Config &config, uint32_t level){
      //boost::mutex::scoped_lock lock(mutex_);
      pos_.x = config.x;
      pos_.y = config.y;
      pos_.z = config.z;
      ori_.x = config.roll;
      ori_.y = config.pitch;
      ori_.z = config.yaw;
      pitch_ = config.preferred_pitch;
      id_ = config.id;
      arm_to_front_ = config.arm_to_front;
      arm_bended_up_ = config.arm_bended_up;
      gripper_downwards_ = config.gripper_downwards;
      move_arm_ = config.move_arm;
      timeout_ = config.timeout;
      method_ = config.method;
      duration_ = config.duration;
      moveArm();
    }
    
    void moveArm(){
      youbot_arm_ik::IkGoal goal;
      goal.timeout = timeout_;
      goal.duration = duration_;
      goal.move_arm = move_arm_;
      if(method_ == 0) goal.name = "closest_ik";
      else if(method_ == 1) goal.name = "preferred_pitch_ik";
      else if(method_ == 2 ) goal.name = "preferred_type_ik";
      else if(method_ == 3) goal.name = "fully_constrained_ik";
      goal.id = id_;
      goal.pitch = pitch_;
      goal.position = pos_;
      goal.orientation = ori_;
      goal.arm_to_front = arm_to_front_;
      goal.arm_bended_up = arm_bended_up_;
      goal.gripper_downwards = gripper_downwards_;
      client_->sendGoal(goal);
      if(client_->waitForResult()){
        actionlib::SimpleClientGoalState state = client_->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        std::vector<double> solution;
        solution = client_->getResult()->solution;
        std::string str = "Solution:[";
        for(size_t i = 0; i < solution.size(); ++i){
          std::ostringstream strs;
          strs << solution[i];
          str += strs.str();
          str += " ";
        }
        str += "]";
        ROS_INFO("%s",str.c_str());
      }
    }
};

int main( int argc, char** argv ){
  ros::init( argc, argv, "youbot_arm_ik_controller" );
  YoubotArmIkController ctr;
  ctr.spin();
  return 0;
}
