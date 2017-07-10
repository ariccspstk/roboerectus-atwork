#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <assert.h>

class IntBox{
public:
  IntBox():nh_("~"), pointer_time_(3.0){
    nh_.param<int>("rate",   rate_, 10);
    nh_.param<int>("window_width",    window_width_, 640);
    nh_.param<int>("window_height",   window_height_, 480);
    nh_.param<double>("min_x", min_x_, -0.1);
    nh_.param<double>("max_x", max_x_, 0.1);
    nh_.param<double>("min_y", min_y_, -0.1);
    nh_.param<double>("max_y", max_y_, 0.1);
    nh_.param<double>("min_z", min_z_, 0.2);
    nh_.param<double>("max_z", max_z_, 0.4);
   
    initMenu(); 
    sub_ = nh_.subscribe("/leap_motion/touch_point", 1, &IntBox::pointCB, this);
    cv::namedWindow("Menu",CV_WINDOW_NORMAL);
    //cv::setWindowProperty("Menu", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
  }

  ~IntBox(){
    sub_.shutdown();
    pub_debug_.shutdown();
    cv::destroyAllWindows();
  }

  void spin(){
    ros::Rate loop_rate(rate_);
    while( nh_.ok() ){
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_debug_;

  int rate_;
  int window_height_;
  int window_width_;
  int btn_size_;
  int menu_id_;
  int menu1_text_id_;
  int menu1_text_no_;
  double min_x_, max_x_;
  double min_y_, max_y_;
  double min_z_, max_z_;

  std::vector<cv::Rect> btns_;
  std::vector<bool> btns_clicked_;
  std::vector<std::string> texts_;
  cv::Point pt_finger_;
  cv::Mat menu_1_, menu_2_;
  bool btn_record_time_;
  ros::Time time_start_;
  const double pointer_time_;

  void initMenu(){
    menu_1_ = cv::Mat(cv::Size(window_width_, window_height_),
      CV_8UC1,cv::Scalar(255));
    menu_2_ = cv::Mat(cv::Size(window_width_, window_height_),
      CV_8UC1,cv::Scalar(255));
    menu_id_ = 0;
    btn_record_time_ = false;
    btn_size_ = 150;
    btns_.clear();
    btns_.resize(5);
    btns_clicked_.clear();
    btns_clicked_.resize(5, false);
    btns_[0] = cv::Rect( window_width_/2 - btn_size_/2, 
      window_height_/2 - btn_size_/2, btn_size_, btn_size_);
    btns_[1] = cv::Rect(btns_[0].x - btn_size_*1.5,btns_[0].y, btn_size_, btn_size_);
    btns_[2] = cv::Rect(btns_[0].x + btn_size_*1.5,btns_[0].y, btn_size_, btn_size_);
    btns_[3] = cv::Rect(window_width_ - btn_size_, btn_size_*0.5, btn_size_*0.5, btn_size_*0.5);
    btns_[4] = cv::Rect(window_width_ - btn_size_*2.5, btn_size_*0.5, btn_size_*0.5, btn_size_*0.5);
    
    //Create Menu1
    
    //Draw center button
    cv::rectangle(menu_1_, btns_[0] , cv::Scalar(0), 5, 8, 0);
    //Draw left button
    cv::rectangle(menu_1_, btns_[1] , cv::Scalar(0), 5, 8, 0);
    std::vector<cv::Point> pts(3);
    pts[0] = cv::Point(window_width_/2 - btn_size_*2, window_height_/2);
    pts[1] = cv::Point(window_width_/2 - btn_size_, window_height_/2 - btn_size_/2);
    pts[2] = cv::Point(window_width_/2 - btn_size_, window_height_/2 + btn_size_/2);
    for(unsigned int i = 0; i < pts.size(); ++i){
      cv::line(menu_1_, pts[i], pts[(i+1)%pts.size()], cv::Scalar(0), 5, 8, 0);
    }
    //Draw right button
    cv::rectangle(menu_1_, btns_[2] , cv::Scalar(0), 5, 8, 0);
    pts[0] = cv::Point(window_width_/2 + btn_size_*2, window_height_/2);
    pts[1] = cv::Point(window_width_/2 + btn_size_, window_height_/2 - btn_size_/2);
    pts[2] = cv::Point(window_width_/2 + btn_size_, window_height_/2 + btn_size_/2);
    for(unsigned int i = 0; i < pts.size(); ++i){
      cv::line(menu_1_, pts[i], pts[(i+1)%pts.size()], cv::Scalar(0), 5, 8, 0);
    }
    cv::rectangle(menu_1_, btns_[3] , cv::Scalar(0), 5, 8, 0);
    cv::rectangle(menu_1_, btns_[4] , cv::Scalar(0), 5, 8, 0);

    //create texts for menu_1
    texts_.clear();
    menu1_text_id_ = 0;
    menu1_text_no_ = 9;
    texts_.resize(menu1_text_no_+1);
    texts_[0]="Follow Me";
    texts_[1]="Goto P1";
    texts_[2]="Goto P2";
    texts_[3]="Goto P3";
    texts_[4]="Goto P4";
    texts_[5]="Goto P5";
    texts_[6]="Goto P6";
    texts_[7]="Goto P7";
    texts_[8]="Goto P8";
    texts_[9]="Goto Charge";
    
    //Create Menu2
    cv::rectangle(menu_2_, btns_[3] , cv::Scalar(0), 5, 8, 0);
    cv::rectangle(menu_2_, btns_[4] , cv::Scalar(0), 5, 8, 0);
    
  }

  void rotateImage(const cv::Mat &input, cv::Mat &output, 
    double alpha, double beta, double gamma, 
    double dx, double dy, double dz, double f){
    alpha = alpha * CV_PI/180.;
    beta =  beta  * CV_PI/180.;
    gamma = gamma * CV_PI/180.;
    // get width and height for ease of use in matrices
    double w = (double)input.cols;
    double h = (double)input.rows;
    // Projection 2D -> 3D matrix
    cv::Mat A1 = (cv::Mat_<double>(4,3) <<
              1, 0, -w/2,
              0, 1, -h/2,
              0, 0,    0,
              0, 0,    1);
    // Rotation matrices around the X, Y, and Z axis
    cv::Mat RX = (cv::Mat_<double>(4, 4) <<
              1,          0,           0, 0,
              0, cos(alpha), -sin(alpha), 0,
              0, sin(alpha),  cos(alpha), 0,
              0,          0,           0, 1);
    cv::Mat RY = (cv::Mat_<double>(4, 4) <<
              cos(beta), 0, -sin(beta), 0,
              0, 1,          0, 0,
              sin(beta), 0,  cos(beta), 0,
              0, 0,          0, 1);
    cv::Mat RZ = (cv::Mat_<double>(4, 4) <<
              cos(gamma), -sin(gamma), 0, 0,
              sin(gamma),  cos(gamma), 0, 0,
              0,          0,           1, 0,
              0,          0,           0, 1);
    // Composed rotation matrix with (RX, RY, RZ)
    cv::Mat R = RX * RY * RZ;
    // Translation matrix
    cv::Mat T = (cv::Mat_<double>(4, 4) <<
             1, 0, 0, dx,
             0, 1, 0, dy,
             0, 0, 1, dz,
             0, 0, 0, 1);
    
    //3D -> 2D matrix
    cv::Mat A2 = (cv::Mat_<double>(3,4) <<
              f, 0, w/2, 0,
              0, f, h/2, 0,
              0, 0,   1, 0);
    
    // Final transformation matrix
    cv::Mat trans = A2 * (T * (R * A1));
    
    //Calculate width and height
    cv::Point2f p1 = cv::Point2f(0, 0);
    cv::Point2f p2 = cv::Point2f(0, h);
    cv::Point2f p3 = cv::Point2f(w, h);
    cv::Point2f p4 = cv::Point2f(w, 0);

    warpPerspectivePoint(p1, p1, trans);
    warpPerspectivePoint(p2, p2, trans);
    warpPerspectivePoint(p3, p3, trans);
    warpPerspectivePoint(p4, p4, trans);

    int offset_x = int(p2.x);
    int offset_y = int(p1.y);
    
    //2D matrix offset
    cv::Mat A3 = (cv::Mat_<double>(3,4) <<
              f, 0, (w/2-offset_x), 0,
              0, f, (h/2-offset_y), 0,
              0, 0,   1, 0);
    trans = A3  * (T * (R * A1));

    //ROS_INFO("%f, %f", p1.x,p1.y);
    //ROS_INFO("%f, %f", p2.x,p2.y);
    
    double width_1  = abs(p1.x - p4.x); 
    double width_2  = abs(p2.x - p3.x); 
    double height_1 = abs(p1.y - p2.y);
    
    int new_width = (width_1 >= width_2)?
      int(width_1 + 0.5):int(width_2 + 0.5);
    int new_height = int(height_1 + 0.5);
    ROS_INFO("%d, %d", new_width, new_height); 
    
    // Apply matrix transformation
    //warpPerspective(input, output, trans, new_size, cv::INTER_LANCZOS4);
    warpPerspective(input, output, trans, cv::Size(new_width, new_height), cv::INTER_LINEAR);
  }

  void warpPerspectivePoint(cv::Point2f input, cv::Point2f& output, cv::Mat m){
    double m11 = m.at<double>(0,0);
    double m12 = m.at<double>(0,1);
    double m13 = m.at<double>(0,2);

    double m21 = m.at<double>(1,0);
    double m22 = m.at<double>(1,1);
    double m23 = m.at<double>(1,2);

    double m31 = m.at<double>(2,0);
    double m32 = m.at<double>(2,1);
    double m33 = m.at<double>(2,2);
    
    double x = input.x;
    double y = input.y;
    output.x = (m11*x+m12*y+m13)/(m31*x+m32*y+m33);
    output.y = (m21*x+m22*y+m23)/(m31*x+m32*y+m33);
  }

  void updateMenu1(cv::Mat& input, std::vector<bool> btns_clicked){
    if(btns_clicked[1]){
      if(++menu1_text_id_ > menu1_text_no_) menu1_text_id_ = 0;
    }
    else if(btns_clicked[2]){
      if(--menu1_text_id_ < 0) menu1_text_id_ = menu1_text_no_;
    }
    else if(btns_clicked[3]){
      menu_id_ = 0;
    }
    else if(btns_clicked[4]){
      menu_id_ = 1;
    }
    else if(btns_clicked[0]){
      //cv::rectangle(input, btns_[2] , cv::Scalar(50), 5, 8, 0);
    }
    // center the text
    cv::Point textOrg((btns_[0].x + btns_[0].width)/2,
    (btns_[0].y + btns_[0].height)/2);
    cv::putText(input, texts_[menu1_text_id_], textOrg, 
      cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 2, cv::Scalar(0), 2, 8);
    
  }

  void updateMenu2(cv::Mat& input, std::vector<bool> btns_clicked){
  }
  
  void updatePointer(cv::Mat& input, double x, double y, double z,
    std::vector<cv::Rect> btns, std::vector<bool>& btns_clicked){
    //Clear btns_clicked to false
    for (std::vector<bool>::iterator it = btns_clicked.begin() ; 
      it != btns_clicked.end(); ++it){
      *it = false;
    }
    double pixelX = window_width_ - y * window_width_;
    double pixelY = window_height_ - x * window_height_;
    int cx = (int) pixelX;
    int cy = (int) pixelY;
    pt_finger_ = cv::Point(cx, cy);
    int max_rad = 20;
    cv::circle(input, pt_finger_, 3, cv::Scalar(0), -1,8,0);

    std::vector<cv::Rect>::iterator it  = btns.begin();
    std::vector<bool>::iterator itt = btns_clicked.begin();
    for( ; it != btns.end(); ++it, ++itt){
      if(pt_finger_.inside(*it)){
        if(!btn_record_time_){
          time_start_ = ros::Time::now();
          btn_record_time_ = true;
        }
        ros::Time time_now = ros::Time::now();
        double time_diff = (time_now - time_start_).toSec();
        int cur_rad = int((time_diff/pointer_time_)*max_rad + 10.5);
        cv::circle(input, pt_finger_, cur_rad, cv::Scalar(0), -1,8,0);
        if( time_diff >= pointer_time_ ){
          btn_record_time_ = false;
          *itt = true;
        }
        return;
      }
    }
    btn_record_time_ = false;
    cv::circle(input, pt_finger_, 10, cv::Scalar(0), 5,8,0);
  }

  void pointCB(const geometry_msgs::PointStampedPtr& msg){
    ros::Time now = ros::Time::now();
    if( ( now - msg->header.stamp ).toSec() > 0.5 ){
      ROS_WARN("intbox: point msg received, but delayed %lf",
        ( now - msg->header.stamp ).toSec() );
      return;
    }
    geometry_msgs::Point point = msg->point;
    //normalize position
    double n_pos_x = (point.x - min_x_)/(max_x_ - min_x_);
    double n_pos_y = (point.y - min_y_)/(max_y_ - min_y_);
    double n_pos_z = (point.z - min_z_)/(max_z_ - min_z_);
   
    n_pos_x = (n_pos_x >= 1.0) ? 1.0: n_pos_x;
    n_pos_y = (n_pos_y >= 1.0) ? 1.0: n_pos_y;
    n_pos_z = (n_pos_z >= 1.0) ? 1.0: n_pos_z;
    
    n_pos_x = (n_pos_x <= 0.0) ? 0.0: n_pos_x;
    n_pos_y = (n_pos_y <= 0.0) ? 0.0: n_pos_y;
    n_pos_z = (n_pos_z <= 0.0) ? 0.0: n_pos_z;

    //ROS_INFO("Pos:(%lf, %lf, %lf)",point.x, point.y, point.z);
    //ROS_INFO("N_Pos:(%lf, %lf, %lf)",n_pos_x, n_pos_y, n_pos_z);
    cv::Mat img = cv::Mat(cv::Size(window_width_, window_height_),
      CV_8UC1,cv::Scalar(255)); 
    if(menu_id_ == 0){
      updatePointer(menu_1_, n_pos_x, n_pos_y, n_pos_z, 
        btns_, btns_clicked_);   
      updateMenu1(menu_1_, btns_clicked_);
      cv::imshow("Menu",menu_1_);
    }
    else if(menu_id_ == 1){
      updatePointer(menu_2_, n_pos_x, n_pos_y, n_pos_z, 
        btns_, btns_clicked_);   
      updateMenu2(menu_2_, btns_clicked_);
      cv::imshow("Menu",menu_2_);
    }
    //rotateImage(menu, menu, -45, 0, 0, 0, 0, 500, 500); 

    cv::waitKey(1);  
  }
};

int main( int argc, char** argv ){
  ros::init( argc, argv, "robot_interaction_box" );
  ROS_INFO("Robot Interaction Box starts");
  IntBox box;
  box.spin();
  return 0;
}
