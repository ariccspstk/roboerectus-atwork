#include <actionlib/client/simple_action_client.h>
#include "dynamic_reconfigure/server.h"
#include <youbot_load_object_belt/LoadAction.h>
#include <youbot_load_object_belt/ControllerConfig.h>
#include <aricc_utils/geometry_utils.h>
#include <geometry_msgs/Pose2D.h>
#include <iostream>
#include <assert.h>
#include <ros/ros.h>

class YoubotLoadObjectBeltController{
  public:
    typedef actionlib::SimpleActionClient<youbot_load_object_belt::LoadAction> ActionClient;
    typedef youbot_load_object_belt::ControllerConfig Config;

    YoubotLoadObjectBeltController():nh_("~"),rate_(5){
      client_.reset( new ActionClient("youbot_load_object_belt_server", true) );
      if(!client_->waitForServer(ros::Duration(5.0))){
        ROS_ERROR("youbot_load_object_belt_server is not excuting");
      }
      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (nh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&YoubotLoadObjectBeltController::configCallback, this, _1, _2);
      srv_->setCallback(f);
      tray_state_.clear();
      tray_state_.resize(3);
      nh_.param<std::string>("tray_1", tray_state_[0], "empty");
      nh_.param<std::string>("tray_2", tray_state_[1], "empty");
      nh_.param<std::string>("tray_3", tray_state_[2], "empty");
      items_.clear();
      std::string item;
      if((nh_.hasParam("item_1"))){
        nh_.getParam("item_1", item);
        items_.push_back(item);
      }
      if((nh_.hasParam("item_2"))){
        nh_.getParam("item_2", item);
        items_.push_back(item);
      }
      if((nh_.hasParam("item_3"))){
        nh_.getParam("item_3", item);
        items_.push_back(item);
      }
    }

    ~YoubotLoadObjectBeltController(){
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
    std::vector<std::string> items_;
    std::vector<std::string> tray_state_;

    void configCallback(Config &config, uint32_t level){
      timeout_    = config.timeout;
      if(config.run) loadObject();
      config.run = false;
    }
    
    void loadObject(){
      youbot_load_object_belt::LoadGoal goal;
      goal.timeout    = timeout_;
      goal.tray_state = tray_state_;
      goal.items      = items_;
      client_->sendGoal(goal);
      items_.clear();
      if(client_->waitForResult()){
        actionlib::SimpleClientGoalState state = client_->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        std::vector<std::string> tray;
        tray = client_->getResult()->tray_state;
        std::string str = "Tray:[";
        for(size_t i = 0; i < tray.size(); ++i){
          str += tray[i];
          str += " ";
        }
        str += "]";
        ROS_INFO("%s",str.c_str());
      }
    }
};

int main( int argc, char** argv ){
  ros::init( argc, argv, "youbot_load_object_belt_controller" );
  YoubotLoadObjectBeltController rt_;
  rt_.spin();
  return 0;
}
