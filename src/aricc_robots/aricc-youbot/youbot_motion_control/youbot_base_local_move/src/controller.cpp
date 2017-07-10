#include <actionlib/client/simple_action_client.h>
#include "dynamic_reconfigure/server.h"
#include <youbot_base_local_move/BaseAction.h>
#include <youbot_base_local_move/ControllerConfig.h>
#include <aricc_utils/geometry_utils.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <assert.h>
#include <ros/ros.h>

class YoubotBaseLocalMoveController{
  public:
    typedef actionlib::SimpleActionClient<youbot_base_local_move::BaseAction> ActionClient;
    typedef youbot_base_local_move::ControllerConfig Config;

    YoubotBaseLocalMoveController():nh_("~"),rate_(5){
      client_.reset( new ActionClient("youbot_base_local_move_server", true) );
      if(!client_->waitForServer(ros::Duration(5.0))){
        ROS_ERROR("youbot_base_local_move_server is not excuting");
      }
      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (nh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&YoubotBaseLocalMoveController::configCallback, this, _1, _2);
      srv_->setCallback(f);

      nh_.param<double>("x",    dest_.x, 5);
      nh_.param<double>("y",    dest_.y, 5);
      double angle;
      nh_.param<double>("theta",angle, 5);
      dest_.theta = aricc_utils::deg2Rad(angle);
    }

    ~YoubotBaseLocalMoveController(){
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
    geometry_msgs::Pose2D dest_;
    uint16_t timeout_;

    void configCallback(Config &config, uint32_t level){
      //boost::mutex::scoped_lock lock(mutex_);
      dest_.x     = config.x;
      dest_.y     = config.y;
      dest_.theta = aricc_utils::deg2Rad(config.theta);
      timeout_ = static_cast<uint16_t>(config.timeout);
      moveBase();
    }
    
    void moveBase(){
      youbot_base_local_move::BaseGoal goal;
      goal.timeout = timeout_;
      goal.destination = dest_;
      client_->sendGoal(goal);
    }
};

int main( int argc, char** argv ){
  ros::init( argc, argv, "youbot_base_local_move_controller" );
  YoubotBaseLocalMoveController rt_;
  rt_.spin();
  return 0;
}
