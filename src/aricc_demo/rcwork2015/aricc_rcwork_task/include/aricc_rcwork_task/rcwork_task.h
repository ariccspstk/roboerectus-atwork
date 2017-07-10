#ifndef RCWORK_TASK_H_
#define RCWORK_TASK_H_

#include <geometry_msgs/Pose.h>
#include <atwork_ros_msgs/BenchmarkState.h>
#include <atwork_ros_msgs/TriggeredConveyorBeltStatus.h>
#include <atwork_ros_msgs/Inventory.h>
#include <atwork_ros_msgs/TaskInfo.h>
#include <aricc_utils/geometry_utils.h>

enum TaskEnum  { kRobotMoveToTable = 20, 
                 kRobotAlignToTable, 
                 kRobotLoadObjectsFromTable,
                 kRobotLoadObjectsFromBelt,
                 kRobotUnloadObjectsToTable, 
                 kRobotUnloadObjectsToHole, 
                 kRobotMoveAway, 
                 kPause,
                 kCancel,
                 kEmpty };

enum StateEnum { ACTIVE = 0, SUCCEEDED, ABORTED };

#define DIR_NORTH 0.0 

class RCworkTask{
public:
  geometry_msgs::Pose location_;
  std::string location_description_;
  double table_height_;
  double align_dist_;
  std::vector<std::string> objects_;
  std::vector<std::string> containers_;
  std::string action_;
  unsigned int pause_;

  TaskEnum now_;
  TaskEnum last_;

  ros::NodeHandle nh_;

  RCworkTask(ros::NodeHandle nh): now_(kRobotMoveToTable), last_(kEmpty),
    pause_(0), align_dist_(0){
    objects_.clear(), containers_.clear();
    nh_ = nh;
  }
  ~RCworkTask(){}

  std::string toString() const{
    switch(now_){
      case kRobotMoveToTable:          return "kRobotMoveToTable";
      case kRobotAlignToTable:         return "kRobotAlignToTable";
      case kRobotLoadObjectsFromTable: return "kRobotLoadObjectsFromTable";
      case kRobotLoadObjectsFromBelt:  return "kRobotLoadObjectsFromBelt";
      case kRobotUnloadObjectsToTable: return "kRobotUnloadObjectsToTable";
      case kRobotUnloadObjectsToHole:  return "kRobotUnLoadObjectsToHole";
      case kPause:                     return "kPause";
      case kCancel:                    return "kCancel";
      case kRobotMoveAway:             return "kRobotMoveAway";

      default:
        ROS_ERROR("BUG: Unhandled State: %u", now_);
        break;
    }
    return "BUG-UNKNOWN";
  }

  bool getNavTask(atwork_ros_msgs::NavigationTask task){
    std::string dir = "";
    
    if(task.orientation.data == 1) dir = "NORTH";
    if(task.orientation.data == 2) dir = "EAST";
    if(task.orientation.data == 3) dir = "SOUTH";
    if(task.orientation.data == 4) dir = "WEST";
    
    if( !getLocation( task.location.description.data, dir ) ) return false;
    pause_ = task.wait_time.data.toSec();
    action_ = "EXIT";
    return true;
  }

  bool getTTaskSrc(atwork_ros_msgs::TransportationTask task){
    if( !getLocation( task.source.description.data, "UNKNOWN" ) ) return false;
    pause_ = 0;
    std::string str = task.source.description.data;
    //ROS_INFO("%s",str.c_str());
    std::string key_1 = "Rotating Table";
    std::string key_2 = "Conveyor Belt";
    std::string key_3 = "Workstation";
    std::string key_4 = "Shelf";
    std::size_t found_1 = str.rfind(key_1);
    std::size_t found_2 = str.rfind(key_2);
    std::size_t found_3 = str.rfind(key_3);
    std::size_t found_4 = str.rfind(key_4);

    if(found_1 != std::string::npos) action_ = "PICKUP_RTT";
    else if(found_2 != std::string::npos) action_ = "PICKUP_CBT";
    else if(found_3 != std::string::npos) action_ = "PICKUP_TABLE";
    else if(found_4 != std::string::npos) action_ = "PICKUP_SHELF";
    else{
      ROS_ERROR("%s is not in action list", str.c_str());
      return false;
    }
 
    objects_.clear();
    containers_.clear();
    std::string name = translateObjectName(task.object.description.data);
    objects_.push_back(name);
    containers_.push_back("Table");
    return true;
  }
  
  bool getTTaskDest(atwork_ros_msgs::TransportationTask task){
    if( !getLocation( task.destination.description.data, "UNKNOWN" ) ) return false;
    pause_ = 0;
    std::string str = task.destination.description.data;
    //ROS_INFO("%s",str.c_str());

    std::string key_1 = "Precision Platform";
    std::string key_2 = "Shelf";
    std::string key_3 = "Workstation";
    std::string key_4 = "Rotating Table";
    std::string key_5 = "Robot";

    std::size_t found_1 = str.rfind(key_1);
    std::size_t found_2 = str.rfind(key_2);
    std::size_t found_3 = str.rfind(key_3);
    std::size_t found_4 = str.rfind(key_4);
    std::size_t found_5 = str.rfind(key_5);

    if(found_1 != std::string::npos) action_ = "PLACE_HOLE";
    else if(found_2 != std::string::npos) action_ = "PLACE_SHELF";
    else if(found_3 != std::string::npos) action_ = "PLACE_TABLE";
    else if(found_4 != std::string::npos) action_ = "PLACE_RTT";
    else if(found_5 != std::string::npos) action_ = "PLACE_ROBOT";
    else{
      ROS_ERROR("%s is not in action list", str.c_str());
      return false; 
    }
    objects_.clear();
    containers_.clear();
    std::string name = translateObjectName(task.object.description.data);
    objects_.push_back(name);

    std::string c = task.container.description.data;
    if( c == "") containers_.push_back("Table");
    else if( c == "Red Container" || c == "Blue Container")
    //uncomment the line below if we are putting into containers
      containers_.push_back(c);
    //uncomment the line below if we are putting onto table
    //  containers_.push_back("Table");
    else ROS_ERROR("%s is not in container list", c.c_str());
    return true;
  }

  std::string translateObjectName(std::string input){
    if(input == "Small Black Alu. Profile") return "F20_20_B";
    else if(input == "Small Grey Alu. Profile")  return "F20_20_G";
    else if(input == "Large Black Alu. Profile") return "S40_40_B";
    else if(input == "Large Grey Alu. Profile")  return "S40_40_G";
    else if(input == "Bolt")                     return "M20_100";
    else if(input == "Small Nut")                return "M20";
    else if(input == "Large Nut")                return "M30";
    else if(input == "Plastic Tube")             return "R20";
    else if(input == "Motor")                    return "Motor";
    else if(input == "Axis")                     return "Axis";
    else if(input == "Bearing Box")              return "Bearing Box";
    else if(input == "Bearing")                  return "Bearing";
    else if(input == "Distance Tube")            return "Distance Tube";
    else return "Unknown";
  }

  bool getLocation( std::string place, std::string dir = "NULL" ){
    XmlRpc::XmlRpcValue list;
    if(!nh_.getParam("locations", list)){
      ROS_ERROR("locations does not exist!");
      return false;
    }
    //ROS_INFO("Found location list, size:%d",list.size());
    ROS_ASSERT( list.getType() == XmlRpc::XmlRpcValue::TypeArray );
    //ROS_INFO("%s", place.c_str());
    if( list.getType() == XmlRpc::XmlRpcValue::TypeArray ){
      for(size_t i = 0; i < list.size(); ++i){
        if( std::string( list[i]["place"] ) == place ){
          location_description_ = place;
          location_.position.x =
          atof(std::string(list[i]["x"]).c_str());
          location_.position.y =
          atof(std::string(list[i]["y"]).c_str());
          location_.position.z =
          atof(std::string(list[i]["z"]).c_str());
          align_dist_ = atof(std::string(list[i]["align"]).c_str());
          table_height_ = atof(std::string(list[i]["height"]).c_str());
          if(dir == "UNKNOWN"){
            getDirection(std::string(list[i]["dir"]));
          }
          else 
            getDirection(dir);
          return true;
        }
      }
      ROS_ERROR("%s is not in location list",place.c_str());
    }
  }

  bool getDirection( std::string dir ){
    double th = 0.0;
    //ROS_INFO("%s", dir.c_str());
    if( dir == "NORTH")      th = 0.0 * M_PI + DIR_NORTH; 
    else if( dir == "WEST")  th = 0.5 * M_PI + DIR_NORTH;
    else if( dir == "SOUTH") th = 1.0 * M_PI + DIR_NORTH; 
    else if( dir == "EAST")  th = 1.5 * M_PI + DIR_NORTH; 
    
    else {
      th = 0.0;
      ROS_WARN("%s is not in direction list, set as default 0", dir.c_str());
    }
    aricc_utils::euler2Quat( 0.0, 0.0, th, location_.orientation );
    return true;
  }

  void printTask(){
    std::ostringstream strs;
    strs << std::endl;
    strs << "Location:" << location_description_ << std::endl;
    strs << "Position:[" << location_.position.x << "," << location_.position.y << "," << location_.position.z << "]" << std::endl;
    strs << "Orientation:[" << location_.orientation.x << "," << location_.orientation.y << "," << location_.orientation.z << ","<< location_.orientation.w << "]" << std::endl;
    strs << "Action:" << action_ << std::endl;
    strs << "Pause:" << pause_ << std::endl;
    strs << "Table Height:" << table_height_ << std::endl;
    strs << "Alignment:" << align_dist_ << std::endl; 
    strs << "Objects:" << objects_.size() << std::endl;
    for(unsigned int j = 0; j < objects_.size(); ++j){
        strs << "[" << objects_.at(j) << ", ";
        strs <<  containers_.at(j) << "]" << std::endl;
    }
    ROS_INFO("%s", strs.str().c_str());
  }
   
private:
};

#endif
