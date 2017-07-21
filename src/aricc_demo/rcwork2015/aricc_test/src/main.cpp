#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Empty.h>

#include <actionlib/client/simple_action_client.h>
#include <youbot_base_global_move/BaseAction.h>
#include <youbot_base_local_move/BaseAction.h>
#include <youbot_base_alignment/BaseAction.h>
#include <youbot_load_object/LoadAction.h>
#include <youbot_unload_object_table/UnloadAction.h>
#include <youbot_unload_object_hole/UnloadAction.h>

#include <aricc_rcwork_task/rcwork_task.h>
#include <aricc_utils/geometry_utils.h>

#include <atwork_ros_msgs/BenchmarkState.h>
#include <atwork_ros_msgs/TriggeredConveyorBeltStatus.h>
#include <atwork_ros_msgs/Inventory.h>
#include <atwork_ros_msgs/TaskInfo.h>

#include <vector>
#include <stdio.h> 
#include <stdlib.h>
#include <math.h>
#include <string>

//const geometry_msgs::Point entry_position_(0,0,0);

bool taskSort( RCworkTask i, RCworkTask j){
  //bool object_size = i.objects_.size() > j.objects_.size();
  return ( i.objects_.size() > j.objects_.size() );
}

class TEST{
public:
  typedef actionlib::SimpleActionClient<youbot_base_global_move::BaseAction> ActionBaseGlobal;
  typedef actionlib::SimpleActionClient<youbot_base_local_move::BaseAction> ActionBaseLocal;
  typedef actionlib::SimpleActionClient<youbot_base_alignment::BaseAction> ActionBaseAlignment;
  typedef actionlib::SimpleActionClient<youbot_load_object::LoadAction> ActionLoadTable;
  typedef actionlib::SimpleActionClient<youbot_unload_object_table::UnloadAction> ActionUnloadTable;
  typedef actionlib::SimpleActionClient<youbot_unload_object_hole::UnloadAction> ActionUnloadHole;
  
  TEST(): pnh_("~"),
    is_base_local_sent_goal_(false),
    is_base_global_sent_goal_(false),
    is_base_alignment_sent_goal_(false),
    is_load_table_sent_goal_(false),
    is_unload_table_sent_goal_(false),
    is_unload_hole_sent_goal_(false),
    refbox_received_(false),
    rate_(5){
  }

  ~TEST(void){
    sub_.shutdown();
  }

  bool init(){
    pnh_.param<double>("rate",        rate_, 5);
    pnh_.param<bool>("test_refbox",   test_refbox_, false);
    pnh_.param<std::string>("refbox_topic", refbox_topic_, "/robot_example_ros/task_info");
    sub_ = nh_.subscribe( refbox_topic_  ,1, &TEST::refCB,this );
    tray_.clear();
    tray_.resize(3,"empty");    
    timeout_ = 180;

    if(test_refbox_) return true;

    client_base_local_.reset( new ActionBaseLocal("youbot_base_local_move_server", true) );
    client_base_global_.reset( new ActionBaseGlobal("youbot_base_global_move_server", true) );
    client_base_alignment_.reset( new ActionBaseAlignment("youbot_base_alignment_server", true) );
    client_load_table_.reset( new ActionLoadTable("youbot_load_object_server", true) );
    client_unload_table_.reset( new ActionUnloadTable("youbot_unload_object_table_server", true) );
    client_unload_hole_.reset( new ActionUnloadHole("youbot_unload_object_hole_server", true) );
   
    unsigned int cnt = 0;
    while(!client_base_global_->waitForServer(ros::Duration(5.0))){
      ROS_WARN("Waiting for youbot_base_global_move actionlib");
      if(++cnt == 5) return false;
    }
    cnt = 0;
    while(!client_base_local_->waitForServer(ros::Duration(5.0))){
      ROS_WARN("Waiting for youbot_base_local_move actionlib");
      if(++cnt == 5) return false;
    }
    cnt = 0;
    while(!client_base_alignment_->waitForServer(ros::Duration(5.0))){
      ROS_WARN("Waiting for youbot_base_alignment actionlib");
      if(++cnt == 5) return false;
    }
    cnt = 0;
    while(!client_load_table_->waitForServer(ros::Duration(5.0))){
      ROS_WARN("Waiting for youbot_load_object actionlib");
      if(++cnt == 5) return false;
    }
    cnt = 0;
    while(!client_unload_table_->waitForServer(ros::Duration(5.0))){
      ROS_WARN("Waiting for youbot_unload_object_table actionlib");
      if(++cnt == 5) return false;
    }
    cnt = 0;
    while(!client_unload_hole_->waitForServer(ros::Duration(5.0))){
      ROS_WARN("Waiting for youbot_unload_object_hole actionlib");
      if(++cnt == 5) return false;
    }

    return true;
  }

  void resetState(){
    client_base_local_->cancelGoal();
    client_base_global_->cancelGoal();
    client_base_alignment_->cancelGoal();
    client_load_table_->cancelGoal();
    client_unload_table_->cancelGoal();
    client_unload_hole_->cancelGoal();
  }

  void spin(){
    ros::Rate r(rate_);
    StateEnum state;
    while( nh_.ok() ){
      //ROS_INFO("Spin");
      r.sleep();
      if( r.cycleTime() > ros::Duration(1.0/rate_) )
        ROS_WARN("TEST: Control desired rate of %.3lfHz... the loop actually took %.4lf seconds", rate_, r.cycleTime().toSec());
      
      ros::spinOnce();
      if(!waitForRefbox()) continue;
      state = taskStep();

      if( state == SUCCEEDED ){
        ros::Time time_now = ros::Time::now(); 
        ROS_INFO("TEST: Succeeded, take %.3lf s", 
          (time_now - time_start_).toSec());
        resetState(); 
        break;
      }
      else if( state == ABORTED ){
        ros::Time time_now = ros::Time::now(); 
        ROS_ERROR("TEST: Aborted, take %.3lf s", 
          (time_now - time_start_).toSec());
        resetState(); 
        break;
      }
    }
  }
  
private:
  boost::shared_ptr<ActionBaseGlobal>    client_base_global_;
  boost::shared_ptr<ActionBaseLocal>     client_base_local_;
  boost::shared_ptr<ActionBaseAlignment> client_base_alignment_;
  boost::shared_ptr<ActionLoadTable>     client_load_table_;
  boost::shared_ptr<ActionUnloadTable>   client_unload_table_;
  boost::shared_ptr<ActionUnloadHole>    client_unload_hole_;
  
  ros::NodeHandle nh_, pnh_;
  ros::Time time_start_;
  ros::Subscriber sub_;
  boost::mutex mutex_;
  double rate_;
  bool test_refbox_;
  double timeout_;
  std::string refbox_topic_;
  bool refbox_received_;
  bool is_base_local_sent_goal_;
  bool is_base_global_sent_goal_;
  bool is_base_alignment_sent_goal_;
  bool is_load_table_sent_goal_;
  bool is_unload_table_sent_goal_;
  bool is_unload_hole_sent_goal_;

  std::vector<RCworkTask> tasks_;
  std::vector<std::string> tray_;

  void refCB(const atwork_ros_msgs::TaskInfo::ConstPtr& msg){
    boost::mutex::scoped_lock lock(mutex_);
    //Always exit to finish whole test
    ROS_INFO("Received Refbox");
    if(refbox_received_) return;
    atwork_ros_msgs::Task t;
    t.type.data = 2;
    t.navigation_task.location.description.data = "Exit"; 
    t.navigation_task.orientation.data = 1;
    t.navigation_task.wait_time.data.fromSec(0);
    atwork_ros_msgs::TaskInfo taskInfo = *msg;
    taskInfo.tasks.push_back(t);
    printTasks(tasks_); 
    if(!loadTasks(taskInfo)){
      ROS_ERROR("Loading task error");
      return;
    }
    ROS_INFO("Shuffling Tasks");
    shuffleTasks(tasks_);
    printTasks(tasks_);
    if(test_refbox_) return;
    refbox_received_ = true;
  }

  bool waitForRefbox(){
    boost::mutex::scoped_lock lock(mutex_);
    if( !refbox_received_ ){
      time_start_ = ros::Time::now();
    }
    return refbox_received_;
  }

  bool loadTasks( atwork_ros_msgs::TaskInfo msg){
    tasks_.clear();
    ROS_INFO("Start loading tasks");
    for(unsigned int i = 0; i < msg.tasks.size(); ++i){
      if(msg.tasks[i].type.data == 1){
        bool result = false;
        RCworkTask t_src(nh_);
        result = t_src.getTTaskSrc(msg.tasks[i].transportation_task);
        if(!result) return false;
        tasks_.push_back(t_src);

        RCworkTask t_dest(nh_);
        result = t_dest.getTTaskDest(msg.tasks[i].transportation_task);
        if(!result) return false;
        tasks_.push_back(t_dest);
      }
      else if(msg.tasks[i].type.data == 2){
        RCworkTask t_nav(nh_);
        bool result = t_nav.getNavTask(msg.tasks[i].navigation_task);
        if(!result) return false;
        tasks_.push_back(t_nav);
      }
      else {
        ROS_ERROR("Got UNKNOWN task type!");
        return false;
      }
    }
    return true;
  }

  void printTasks( std::vector<RCworkTask> tasks){
    ROS_INFO("Got %lu tasks", tasks.size());
    for(unsigned int i = 0; i < tasks.size(); ++i){
      ROS_INFO("Task_%d",i);
      tasks.at(i).printTask();
    }
  }

  bool isSameLocation(RCworkTask task1,RCworkTask task2){
    if(task1.location_description_ == task2.location_description_)
      return true;
    return false;
  }

  bool hasObject(RCworkTask task, std::string object){
    for(unsigned int i = 0; i < task.objects_.size(); ++i){
      if(task.objects_.at(i) == object) return true;
    }
    return false;
  }

  void mergeTasksByLocation(std::vector<RCworkTask>& tasks, 
    int max_object = 3){
    std::vector<RCworkTask>::iterator it = tasks.begin();
    for( ; it != tasks.end(); ){
      std::vector<RCworkTask>::iterator itt = it+1;
      for(; itt != tasks.end(); ){
        if( isSameLocation( *itt, *it) ){
          it->objects_.push_back(itt->objects_.at(0));
          it->containers_.push_back(itt->containers_.at(0));
          itt = tasks.erase(itt);
          if(it->objects_.size() >= max_object) break;
        }
        else ++itt;      
      }
      ++it;
    }
  }

  void addTasks(std::vector<RCworkTask>& dest_tasks, 
    std::vector<RCworkTask> src_tasks){
    for(unsigned int i = 0; i < src_tasks.size(); ++i)
      dest_tasks.push_back(src_tasks.at(i));
  }

  void shuffleTasks(std::vector<RCworkTask>& tasks){
    std::vector<RCworkTask> pickup_tasks;
    std::vector<RCworkTask> place_tasks;
    std::vector<RCworkTask> nav_tasks;

    pickup_tasks.clear();
    place_tasks.clear();
    nav_tasks.clear();

    for(unsigned int i = 0; i < tasks.size(); ++i){
      std::string key_1 = "PICKUP";
      std::string key_2 = "PLACE";
      std::string key_3 = "EXIT";
      std::size_t found_1 = tasks.at(i).action_.rfind(key_1);
      std::size_t found_2 = tasks.at(i).action_.rfind(key_2);
      std::size_t found_3 = tasks.at(i).action_.rfind(key_3);

      if(found_1 != std::string::npos){
        pickup_tasks.push_back(tasks.at(i));
      }
      else if( found_2 != std::string::npos ){
        place_tasks.push_back(tasks.at(i));
      }
      else if( found_3 != std::string::npos ){
        nav_tasks.push_back(tasks.at(i));
      }
    }
    //ROS_INFO("PICKUP Tasks");
    //printTasks(pickup_tasks);
    //printTasks(place_tasks);
    //return;
    
    //Merge tasks with same source location
    mergeTasksByLocation(pickup_tasks);
    //mergeTasks(place_tasks);
    //printTasks(pickup_tasks);
    //printTasks(place_tasks);

    //Sort pickup tasks by quantity of object
    std::sort(pickup_tasks.begin(), pickup_tasks.end(),taskSort);
    //std::sort(place_tasks.begin(), place_tasks.end(),taskSort);
    //printTasks(pickup_tasks);
    //printTasks(place_tasks);
 
    tasks.clear();
    std::vector<RCworkTask>::iterator it = pickup_tasks.begin();
    for( ; it != pickup_tasks.end(); ){
      tasks.push_back(*it);
      std::vector<RCworkTask>temp_tasks;
      temp_tasks.clear();
      for(unsigned int i = 0; i < it->objects_.size(); ++i){
        std::vector<RCworkTask>::iterator itt =place_tasks.begin();
        for(; itt != place_tasks.end(); ){
          if( hasObject(*itt, it->objects_.at(i)) ){
            //ROS_INFO("Current got tasks:%lu", tasks.size());
            temp_tasks.push_back(*itt);
            //tasks.push_back(*itt);
            itt = place_tasks.erase(itt);
            break;
          }
          else ++itt;
        }
      }
      mergeTasksByLocation(temp_tasks);
      addTasks(tasks,temp_tasks);
      ++it;
    }
    
    it = nav_tasks.begin();
    for( ; it != nav_tasks.end(); ){
      tasks.push_back(*it);
      ++it;
    }
    
    //printTasks(nav_tasks); 
  }

  StateEnum moveBaseGlobal(geometry_msgs::Pose pose){
    if(!is_base_global_sent_goal_){
      youbot_base_global_move::BaseGoal goal;
      goal.timeout = 40;
      goal.destination = pose;
      client_base_global_->sendGoal(goal);
      is_base_global_sent_goal_ = true;
    }
    if( client_base_global_->getState().toString() == "SUCCEEDED" ) {
      is_base_global_sent_goal_ = false;
      return SUCCEEDED;
    }
    else if( client_base_global_->getState().toString() == "ABORTED" ) {
      is_base_global_sent_goal_ = false;
      return ABORTED;
    }
    return ACTIVE;
  }

  StateEnum moveBaseLocal( geometry_msgs::Pose2D pose){
    if(!is_base_local_sent_goal_){
      youbot_base_local_move::BaseGoal goal;
      goal.timeout = 30;
      goal.destination = pose;
      client_base_local_->sendGoal(goal);
      is_base_local_sent_goal_ = true;
    }
    if( client_base_local_->getState().toString() == "SUCCEEDED" ){
      is_base_local_sent_goal_ = false;
      return SUCCEEDED;
    }
    else if( client_base_local_->getState().toString() == "ABORTED" ){
      is_base_local_sent_goal_ = false;
      return ABORTED;
    }
    else return ACTIVE;
  }

  StateEnum moveBaseAlignment( double dist ){
    if(!is_base_alignment_sent_goal_){
      youbot_base_alignment::BaseGoal goal;
      goal.timeout = 30;
      goal.distance = dist;
      client_base_alignment_->sendGoal(goal);
      is_base_alignment_sent_goal_ = true;
    }
    if( client_base_alignment_->getState().toString() == "SUCCEEDED" ){
      is_base_alignment_sent_goal_ = false;
      return SUCCEEDED;
    }
    else if( client_base_alignment_->getState().toString() == "ABORTED" ){
      is_base_alignment_sent_goal_ = false;
      return ABORTED;
    }
    else return ACTIVE;
  }

  StateEnum loadTable( std::vector<std::string> items,
    std::vector<std::string>& tray,
    double table_height){
    if(!is_load_table_sent_goal_){
      youbot_load_object::LoadGoal goal;
      goal.timeout = timeout_;
      goal.tray_state = tray;
      goal.table_height = table_height;
      goal.items = items;
      client_load_table_->sendGoal(goal);
      is_load_table_sent_goal_ = true;
    }
    if( client_load_table_->getState().toString() == "SUCCEEDED" ){
      is_load_table_sent_goal_ = false;
      tray = client_load_table_->getResult()->tray_state;
      std::string str = "Tray:[";
      for(size_t i = 0; i < tray.size(); ++i){
        str += tray[i];
        str += " ";
      }
      str += "]";
      ROS_INFO("%s",str.c_str());
      return SUCCEEDED;
    }
    else if( client_load_table_->getState().toString() == "ABORTED" ){
      is_load_table_sent_goal_ = false;
      tray = client_load_table_->getResult()->tray_state;
      std::string str = "Tray:[";
      for(size_t i = 0; i < tray.size(); ++i){
        str += tray[i];
        str += " ";
      }
      str += "]";
      ROS_INFO("%s",str.c_str());
      return ABORTED;
    }
    else return ACTIVE;
  }

  StateEnum loadBelt( std::vector<std::string> items,
    std::vector<std::string>& tray){
    return SUCCEEDED;
  }

  StateEnum unloadTable( std::vector<std::string> items,
    std::vector<std::string> containers,
    std::vector<std::string>& tray,
    double table_height ){
    if(!is_unload_table_sent_goal_){
      youbot_unload_object_table::UnloadGoal goal;
      goal.timeout = timeout_;
      goal.items = items;
      goal.tray_state = tray;
      goal.table_height = table_height;
      goal.containers = containers;
      client_unload_table_->sendGoal(goal);
      is_unload_table_sent_goal_ = true;
    }
    if( client_unload_table_->getState().toString() == "SUCCEEDED" ){
      is_unload_table_sent_goal_ = false;
      tray = client_unload_table_->getResult()->tray_state;
      std::string str = "Tray:[";
      for(size_t i = 0; i < tray.size(); ++i){
        str += tray[i];
        str += " ";
      }
      str += "]";
      ROS_INFO("%s",str.c_str());
      return SUCCEEDED;
    }
    else if( client_unload_table_->getState().toString() == "ABORTED" ){
      is_unload_table_sent_goal_ = false;
      tray = client_unload_table_->getResult()->tray_state;
      std::string str = "Tray:[";
      for(size_t i = 0; i < tray.size(); ++i){
        str += tray[i];
        str += " ";
      }
      str += "]";
      ROS_INFO("%s",str.c_str());
      return ABORTED;
    }
    else return ACTIVE;
  }

  StateEnum unloadHole( std::vector<std::string> items, 
    std::vector<std::string>& tray,
    double table_height ){
    if(!is_unload_hole_sent_goal_){
      youbot_unload_object_hole::UnloadGoal goal;
      goal.timeout = timeout_;
      goal.items = items;
      goal.tray_state = tray;
      goal.table_height = table_height;

      client_unload_hole_->sendGoal(goal);
      is_unload_hole_sent_goal_ = true;
    }
    if( client_unload_hole_->getState().toString() == "SUCCEEDED" ){
      is_unload_hole_sent_goal_ = false;
      tray = client_unload_hole_->getResult()->tray_state;
      std::string str = "Tray:[";
      for(size_t i = 0; i < tray.size(); ++i){
        str += tray[i];
        str += " ";
      }
      str += "]";
      ROS_INFO("%s",str.c_str());
      return SUCCEEDED;
    }
    else if( client_unload_hole_->getState().toString() == "ABORTED" ){
      is_unload_hole_sent_goal_ = false;
      tray = client_unload_hole_->getResult()->tray_state;
      std::string str = "Tray:[";
      for(size_t i = 0; i < tray.size(); ++i){
        str += tray[i];
        str += " ";
      }
      str += "]";
      ROS_INFO("%s",str.c_str());
      return ABORTED;
    }
    else return ACTIVE;
  }

  void updateTaskList(){
    tasks_.erase(tasks_.begin());
    ROS_INFO("Task list left %lu tasks",tasks_.size());
  }

  StateEnum taskStep(){
    if( tasks_[0].now_ != tasks_[0].last_ )
      ROS_INFO( "state now:%s",tasks_[0].toString().c_str() );
    tasks_[0].last_ = tasks_[0].now_;

    StateEnum state;
    geometry_msgs::Pose2D basePose;

    switch( tasks_[0].now_ ){
      case kRobotMoveToTable:
       //if we want to go to 0cm tables
       //if(tasks_[0].action_ == "PICKUP_CBT" ||
       //  tasks_[0].action_ == "PICKUP_RTT"){
       //  tasks_[0].now_ = kPause;
       //if we don't wna go to 0cm tables 
       if(tasks_[0].action_ == "PICKUP_CBT"  ||
         tasks_[0].action_ == "PICKUP_RTT"   ||
         tasks_[0].action_ == "PICKUP_SHELF" ||
         (tasks_[0].action_ == "PICKUP_TABLE" &&
         tasks_[0].table_height_ == 0.0)){
        tasks_[0].now_ = kPause;

       }
       else{
         state = moveBaseGlobal(tasks_[0].location_);
         //state = SUCCEEDED; 
         if( state == SUCCEEDED ){
           if(tasks_[0].action_ == "EXIT") tasks_[0].now_ = kPause;
           else tasks_[0].now_ = kRobotAlignToTable;
          }
          else if( state == ABORTED ) tasks_[0].now_ = kCancel;
       }
      break;

      case kRobotAlignToTable:
        if( tasks_[0].align_dist_ < 0 ) state = SUCCEEDED;
        else state = moveBaseAlignment(tasks_[0].align_dist_);
        
        if( state == SUCCEEDED ){
           if(tasks_[0].action_ == "PICKUP_TABLE") {
             tasks_[0].now_ = kRobotLoadObjectsFromTable;
           }
           if(tasks_[0].action_ == "PICKUP_CBT") {
             //tasks_[0].now_ = kRobotLoadObjectsFromBelt;
             //tasks_[0].now_ = kRobotLoadObjectsFromTable;
             tasks_[0].now_ = kRobotMoveAway;
           }
           if(tasks_[0].action_ == "PICKUP_SHELF") {
             //tasks_[0].now_ = kRobotLoadObjectsFromBelt;
             tasks_[0].now_ = kRobotLoadObjectsFromTable;
           }
           if(tasks_[0].action_ == "PICKUP_RTT") {
             //tasks_[0].now_ = kRobotLoadObjectsFromBelt;
             //tasks_[0].now_ = kRobotLoadObjectsFromTable;
             tasks_[0].now_ = kRobotMoveAway;
           }
           if(tasks_[0].action_ == "PLACE_TABLE") {
             tasks_[0].now_ = kRobotUnloadObjectsToTable;
           }
           if(tasks_[0].action_ == "PLACE_RTT") {
             tasks_[0].now_ = kRobotUnloadObjectsToTable;
           }
           if(tasks_[0].action_ == "PLACE_SHELF") {
             tasks_[0].now_ = kRobotUnloadObjectsToTable;
           }
           if(tasks_[0].action_ == "PLACE_HOLE") {
             tasks_[0].now_ = kRobotUnloadObjectsToHole;
           }
           if(tasks_[0].action_ == "PLACE_ROBOT") {
             tasks_[0].now_ = kRobotMoveAway;
           }
        }
        else if( state == ABORTED ) tasks_[0].now_ = kCancel;
        
        //if(state == SUCCEEDED) tasks_[0].now_ = kPause;
      break;

      case kRobotLoadObjectsFromTable:
        state = loadTable(tasks_[0].objects_, tray_, tasks_[0].table_height_);
        if( state == SUCCEEDED ){
          tasks_[0].now_ = kRobotMoveAway;
        }
        else if( state == ABORTED ) tasks_[0].now_ = kCancel;
      break;

      case kRobotLoadObjectsFromBelt:
        state = loadBelt(tasks_[0].objects_, tray_);
        if( state == SUCCEEDED ){
          tasks_[0].now_ = kRobotMoveAway;
        }
        else if( state == ABORTED ) tasks_[0].now_ = kCancel;
      break;

      case kRobotUnloadObjectsToTable:
        state = unloadTable(tasks_[0].objects_, tasks_[0].containers_, tray_,tasks_[0].table_height_);
        if( state == SUCCEEDED ){
          tasks_[0].now_ = kRobotMoveAway;
        }
        else if( state == ABORTED ) tasks_[0].now_ = kCancel;
      break;

      case kRobotUnloadObjectsToHole:
        state = unloadHole(tasks_[0].objects_, tray_, tasks_[0].table_height_);
        if( state == SUCCEEDED ){
          tasks_[0].now_ = kRobotMoveAway;
        }
        else if( state == ABORTED ) tasks_[0].now_ = kCancel;
      break;

      case kRobotMoveAway:
        basePose.x = -0.03;
        basePose.y = 0.0;
        basePose.theta = 0.0;
        state = moveBaseLocal(basePose);
        if(state == SUCCEEDED) tasks_[0].now_ = kPause;
        else if(state == ABORTED) tasks_[0].now_ = kCancel;
      break;

      case kPause:
        sleep(tasks_[0].pause_);
        updateTaskList();
        if(tasks_.size() == 0){ 
          return SUCCEEDED;
        }
      break;

      case kCancel:
        ROS_ERROR("step aborted, cancel current task");
        sleep(tasks_[0].pause_);
        updateTaskList();
        if(tasks_.size() == 0){ 
          refbox_received_ = false;
          return ABORTED;
        }
      break;

      default:
      break;
    }
    return ACTIVE;
  }

};

int main(int argc, char ** argv){
  ros::init(argc,argv,"aricc_test");
  TEST test;
  if( !test.init() ){
    ROS_ERROR("robocup @work competition initial failed!");
    return 0;
  }
  ROS_INFO("Starting aricc_test");
  test.spin();
  return 0;
}
