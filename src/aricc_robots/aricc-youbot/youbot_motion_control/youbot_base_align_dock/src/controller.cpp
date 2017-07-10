#include <actionlib/client/simple_action_client.h>
#include "dynamic_reconfigure/server.h"
#include <youbot_base_align_dock/BaseAction.h>
#include <youbot_base_align_dock/ControllerConfig.h>
#include <aricc_utils/geometry_utils.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <assert.h>
#include <ros/ros.h>

class YoubotBaseAlignDockController{
  public:
    typedef actionlib::SimpleActionClient<youbot_base_align_dock::BaseAction> ActionClient;
    typedef youbot_base_align_dock::ControllerConfig Config;

    YoubotBaseAlignDockController():nh_("~"),rate_(5){
      client_.reset( new ActionClient("youbot_base_align_dock_server", true) );
      if(!client_->waitForServer(ros::Duration(5.0))){
        ROS_ERROR("youbot_base_align_dock_server is not excuting");
      }
      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (nh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&YoubotBaseAlignDockController::configCallback, this, _1, _2);
      srv_->setCallback(f);
    }

    ~YoubotBaseAlignDockController(){
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
    double timeout_;
    double dist_;

    void configCallback(Config &config, uint32_t level){
      timeout_    = config.timeout;
      moveBase();
    }
    
    void moveBase(){
      youbot_base_align_dock::BaseGoal goal;
      goal.timeout  = timeout_;
      client_->sendGoal(goal);
      /*
      if(client_->waitForResult()){
        actionlib::SimpleClientGoalState state = client_->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
      }*/
    }
};

int main( int argc, char** argv ){
  ros::init( argc, argv, "youbot_base_align_dock_controller" );
  YoubotBaseAlignDockController rt_;
  rt_.spin();
  return 0;
}
