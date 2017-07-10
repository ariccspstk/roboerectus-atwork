#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace aricc_control{

class LowPassController{
private:
  double t_; //sec
  double dt_; //ms
  double x_, y_, theta_, eps_;
  double rate_;
  std::string pub_topic_, sub_topic_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::NodeHandle nh_, pnh_;

  void velCB(const geometry_msgs::Twist::ConstPtr& msg){
    double x     = msg->linear.x;
    double y     = msg->linear.y;
    double theta = msg->angular.z;
    updateFilter(x, x_);
    updateFilter(y, y_);
    updateFilter(theta, theta_);
    
    geometry_msgs::Twist t;
    t.linear.x = x_;
    t.linear.y = y_;
    t.angular.z = theta_;
    pub_.publish(t);
  }

  void updateFilter(double x, double& y){
    double res = y + (x - y) * (dt_/(dt_+t_));
    //ROS_INFO("Res:%lf",res);
    if ( fabs(res) <= eps_ ) y = 0.0;
    else y = res;
  }

  void setGains(double t, double dt, double eps){
    t_ = t;
    dt_ = dt;
    eps_ = eps;
  }

  void resetFilter(){
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
  }

public:
  LowPassController():pnh_("~"){
    resetFilter();
    pnh_.param<double>("rate", rate_, 20);
    pnh_.param<double>("t", t_, 0.2);
    pnh_.param<double>("dt", dt_, 0.1);
    pnh_.param<double>("eps", eps_, 0.025);
    pnh_.param<std::string>("pub_topic", pub_topic_, "cmd_vel_lowpass");
    pnh_.param<std::string>("sub_topic", sub_topic_, "cmd_vel");
    pub_ = nh_.advertise<geometry_msgs::Twist>( pub_topic_, 1, true);
    sub_ = nh_.subscribe( sub_topic_, 1, &LowPassController::velCB, this);
  }

  ~LowPassController(){
    sub_.shutdown();
  }

  void spin(){
    ros::spin();
    //ros::Rate r(rate_); //Input and output at the same time... (in Hz)
    //while(nh_.ok()){
    //  ros::spinOnce();
    //  r.sleep();
    //}
  }
};

}

int main(int argc, char **argv){
  ros::init(argc, argv, "aricc_lowpass_filter");
  aricc_control::LowPassController lpc;
  lpc.spin();
  return 0;
}
