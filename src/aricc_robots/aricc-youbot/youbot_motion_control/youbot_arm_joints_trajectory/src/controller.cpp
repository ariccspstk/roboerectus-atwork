#include <actionlib/client/simple_action_client.h>
#include "dynamic_reconfigure/server.h"
#include <youbot_arm_joints_trajectory/TrajectoryAction.h>
#include <youbot_arm_joints_trajectory/ControllerConfig.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <assert.h>
#include "ros/ros.h"

class YoubotArmJointsTrajectoryController{
  public:
    typedef actionlib::SimpleActionClient<youbot_arm_joints_trajectory::TrajectoryAction> ActionClient;
    typedef youbot_arm_joints_trajectory::ControllerConfig Config;
    typedef std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory;

    YoubotArmJointsTrajectoryController():pnh_("~"),rate_(5){
      client_.reset( new ActionClient("youbot_arm_joints_trajectory_server", true) );
      if(!client_->waitForServer(ros::Duration(5.0))){
        ROS_ERROR("youbot_arm_joints_trajectory_server is not excuting");
      }
      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&YoubotArmJointsTrajectoryController::configCallback, this, _1, _2);
      srv_->setCallback(f);
      sub_ = nh_.subscribe("/joint_states"  ,1, &YoubotArmJointsTrajectoryController::jointsCB,this);
      goal_.points.clear();
      joints_state_.clear();

    }

    ~YoubotArmJointsTrajectoryController(){
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
    ros::NodeHandle pnh_;
    unsigned int rate_;
    boost::shared_ptr<ActionClient> client_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
    std::vector<double> joints_state_;
    youbot_arm_joints_trajectory::TrajectoryGoal goal_;
    double timeout_;
    ros::Subscriber sub_;
    std::vector<std::string> trajectory_name_list_;
    std::vector<trajectory> trajectory_list_;
    
    void printTrajectory(trajectory traj){
    ROS_INFO("Trajectory has %lu points.", traj.size());
    for(unsigned int i = 0; i < traj.size(); ++i){
      std::ostringstream strs;
      std::string str = "Point_";
      strs << i << ": ";
      for(unsigned int j = 0; j < traj[i].positions.size(); ++j){
        strs << traj[i].positions[j] << " ";
      }
      strs << "Duration:" << traj[i].time_from_start;
      str += strs.str();
      ROS_INFO("%s", str.c_str());
    }
  }   
 
    void configCallback(Config &config, uint32_t level){
      //boost::mutex::scoped_lock lock(mutex_);
      if(joints_state_.empty()) return;
      timeout_ = config.timeout;
      config.joint0 = joints_state_[0];
      config.joint1 = joints_state_[1];
      config.joint2 = joints_state_[2];
      config.joint3 = joints_state_[3];
      config.joint4 = joints_state_[4];
      
      if(config.record){
        config.record = false;
        trajectory_msgs::JointTrajectoryPoint segment;
        segment.positions.resize(5);
        for(unsigned int i = 0; i < 5; ++i)
          segment.positions[i] = joints_state_[i];
        segment.time_from_start = ros::Duration(5.0);
        goal_.points.push_back(segment);
        ROS_INFO("Recorded the %lu segments: [%lf, %lf, %lf, %lf, %lf]", goal_.points.size(), segment.positions[0], segment.positions[1],
       segment.positions[2], segment.positions[3], segment.positions[4]); 
      }
      if(config.clear){
        config.clear = false;
        goal_.points.clear();
      }
      if(config.run){
        config.run = false;
        moveArm();
      }

      if(config.load){
        config.load = false;
        goal_.points.clear();
        loadTrajectoryList();
        trajectory_list_.clear();
        for(unsigned int i = 0; i < trajectory_name_list_.size(); ++i){
          trajectory traj;
          loadArmTrajectory(trajectory_name_list_[i], traj);
          trajectory_list_.push_back(traj);
        }
      }
      
      if(config.id < trajectory_list_.size() && !trajectory_list_.empty()){
        goal_.points = trajectory_list_[config.id];
        config.traj_name = trajectory_name_list_[config.id]; 
      }
    }

    bool loadTrajectoryList(){
      try{
        XmlRpc::XmlRpcValue list;
        if( !pnh_.hasParam("trajectory_list") ){
          ROS_ERROR("Cannot find trajectory_list");
          return false;
        }
        pnh_.getParam( "trajectory_list", list );
        ROS_INFO("Found trajectory list: %d", list.size());
        ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        trajectory_name_list_.clear();
        std::string str = "Loaded: \n";
        for(size_t i = 0; i < list.size(); ++i){
          ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
          std::string value = static_cast<std::string>(list[i][0]);
          str += "[";
          str += value;
          str += "]";
          str += "\n";
          trajectory_name_list_.push_back( value.c_str() );
        }
        ROS_INFO("%s", str.c_str());
      }
      catch(ros::Exception e){
        ROS_ERROR("%s", e.what());
        return false;
      }
      return true;
    }
    
    bool loadArmTrajectory(std::string name, trajectory& traj){
      try{
        XmlRpc::XmlRpcValue list;
        if( !pnh_.hasParam(name) ){
          ROS_ERROR("Cannot find %s", name.c_str());
          return false;
        }
        pnh_.getParam( name, list );
        ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_INFO("%s has %d waypoints:",name.c_str(), list.size());
        traj.clear();
        traj.resize(list.size());

        for(unsigned int i = 0; i < list.size(); ++i){
          traj[i].positions.resize(5);
          std::string d = static_cast<std::string>(list[i]["duration"]);
          traj[i].time_from_start = ros::Duration(atof(d.c_str()));
          XmlRpc::XmlRpcValue joint_list = list[i]["joints"];
          //ROS_INFO("%d joint list found", joint_list.size());
          for(unsigned int j = 0; j < joint_list.size(); ++j){
            std::string value = static_cast<std::string>(joint_list[j]);
            traj[i].positions[j] = atof(value.c_str());
          }
        }      
      }
      catch(ros::Exception e){
        ROS_ERROR("%s", e.what());
        return false;
      }
      printTrajectory(traj); 
      return true;
    }

    void jointsCB(const sensor_msgs::JointState::ConstPtr& msg){
      boost::mutex::scoped_lock lock(mutex_);
      joints_state_.clear();
      if(msg->position.size() != 7 && msg->name[0] == "arm_joint_1"){ 
        ROS_WARN("It should be 7 joints, received %lu joints!",
        msg->position.size());
      return;
      }
      for(size_t i = 0; i < msg->position.size(); ++i)
      joints_state_.push_back(msg->position[i]);
    }
    
    void moveArm(){
      ROS_INFO("Moving Arm");
      printTrajectory(goal_.points);
      goal_.timeout = timeout_;
      client_->sendGoal(goal_);
      if(client_->waitForResult()){
        actionlib::SimpleClientGoalState state = client_->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
      }
      goal_.points.clear();
    }
};

int main( int argc, char** argv ){
  ros::init( argc, argv, "youbot_arm_joints_trajectory_controller" );
  YoubotArmJointsTrajectoryController rt_;
  rt_.spin();
  return 0;
}
