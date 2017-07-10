#include "aricc_2d_vision/contour_finder.h"

namespace aricc_2d_vision{

  void ContourFinder::onInit(){
    ConnectionBasedNodelet::onInit();
    pnh_->param("debug", debug_, false);
    pnh_->param("draw_contours", draw_contours_, false);
    pnh_->param("draw_convex",   draw_convex_, false);
    pnh_->param("draw_poly",     draw_poly_, false);
   
    pnh_->param("noise_size", noise_size_, 10.0);
    pnh_->param("min_size", min_size_, 100.0);
    pnh_->param("min_dist", min_dist_, 20.0);
    pnh_->param("close", close_, true);
    pnh_->param("epsilon", epsilon_, 3.0);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &ContourFinder::configCallback, this, _1, _2);
    srv_->setCallback (f);
    if(debug_){
      pub_debug_image_ = advertise<sensor_msgs::Image>(*pnh_, "debug", 1);
    }
    pub_contours_ = 
      advertise<aricc_vision_msgs::ContourArray>(*pnh_,"output/contours", 1);
    pub_convex_contours_ = 
      advertise<aricc_vision_msgs::ContourArray>(*pnh_,"output/convex", 1);
    pub_poly_contours_ = 
      advertise<aricc_vision_msgs::ContourArray>(*pnh_,"output/poly", 1);
  }

  void ContourFinder::subscribe(){
    sub_ = pnh_->subscribe("input", 1, &ContourFinder::execute, this);
  }

  void ContourFinder::unsubscribe(){
    sub_.shutdown();
  }
  
  void ContourFinder::configCallback(
    Config &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    noise_size_ = config.noise_size;
    min_size_   = config.min_size;
    min_dist_   = config.min_dist;
    debug_      = config.debug;
    close_      = config.close;
    epsilon_    = config.epsilon;
    draw_contours_ = config.draw_contours;
    draw_convex_ = config.draw_convex;
    draw_poly_ = config.draw_poly;
   
  }

  void ContourFinder::mergeContours(std::vector<cv::Point> src, 
                            std::vector<cv::Point>& dst){
    for (size_t i = 0; i < src.size(); ++i) {
        dst.push_back(src.at(i));
    }
  }

  int ContourFinder::onBorder(std::vector<cv::Point> contour,
                              int width, int height, double dist = 5.0){
    std::vector<cv::Point>::iterator it = contour.begin();
    for(; it != contour.end(); ++it){
      if(it->x < dist || it->x > (width-dist)) return 1;
      if(it->y < dist || it->y > (height-dist)) return 1;
    }
    return 0;
  }
  
  void ContourFinder::pubResult(std_msgs::Header header){
    aricc_vision_msgs::ContourArray msg_contour_array;
    aricc_vision_msgs::ContourArray msg_convex_array;
    aricc_vision_msgs::ContourArray msg_poly_array;

    std::vector <std::vector<cv::Point> >::iterator it 
      = contours_.begin();
    for( ; it !=contours_.end(); ++it){
      aricc_vision_msgs::Contour msg;
      for(size_t i = 0; i < it->size(); ++i){
        geometry_msgs::Point p;
        p.x = it->at(i).x;
        p.y = it->at(i).y;
        msg.points.push_back(p);
      }
      msg_contour_array.contours.push_back(msg);
    }

    it = convex_contours_.begin();
    for( ; it !=convex_contours_.end(); ++it){
      aricc_vision_msgs::Contour msg;
      for(size_t i = 0; i < it->size(); ++i){
        geometry_msgs::Point p;
        p.x = it->at(i).x;
        p.y = it->at(i).y;
        msg.points.push_back(p);
      }
      msg_convex_array.contours.push_back(msg);
    }

    it = poly_contours_.begin();
    for( ; it !=poly_contours_.end(); ++it){
      aricc_vision_msgs::Contour msg;
      for(size_t i = 0; i < it->size(); ++i){
        geometry_msgs::Point p;
        p.x = it->at(i).x;
        p.y = it->at(i).y;
        msg.points.push_back(p);
      }
      msg_poly_array.contours.push_back(msg);
    }
    
    msg_contour_array.header = header;
    msg_convex_array.header = header;
    msg_poly_array.header = header;
    msg_contour_array.header.stamp = ros::Time::now();
    msg_convex_array.header.stamp = ros::Time::now();
    msg_poly_array.header.stamp = ros::Time::now();
    pub_contours_.publish(msg_contour_array);
    pub_convex_contours_.publish(msg_convex_array);
    pub_poly_contours_.publish(msg_poly_array);

    if(debug_){ 
      //cv::RNG rng(12345);
      //cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      cv::Scalar red = cv::Scalar( 0, 0, 255 );
      cv::Scalar green = cv::Scalar( 0, 255, 0 );
      cv::Scalar blue = cv::Scalar( 255, 0, 0 );
      cv::Mat drawing = cv::Mat::zeros(cv::Size(img_width_, img_height_), CV_8UC3);
      if(draw_contours_){ 
        for(size_t i = 0; i < contours_.size(); ++i){
          cv::drawContours(drawing, contours_, i, red, 2, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        }
      }
      if(draw_convex_){
        for(size_t i = 0; i < convex_contours_.size(); ++i){
          cv::drawContours(drawing, convex_contours_, i, green, 2, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        }
      }
      if(draw_poly_){
        for(size_t i = 0; i <poly_contours_.size(); ++i){
        cv::drawContours(drawing, poly_contours_, i, blue, 2, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        }
      }
      pub_debug_image_.publish(cv_bridge::CvImage(
                               header,
                               sensor_msgs::image_encodings::BGR8,
                               drawing).toImageMsg());
    }
  }
  
  void ContourFinder::execute(
    const sensor_msgs::Image::ConstPtr& image_msg){
    boost::mutex::scoped_lock lock(mutex_);
    contours_.clear();
    convex_contours_.clear();
    poly_contours_.clear();
    hierarchy_.clear();

    cv::Mat input = cv_bridge::toCvCopy(
      image_msg, sensor_msgs::image_encodings::MONO8)->image;
    img_width_ = input.cols;
    img_height_ = input.rows;

    cv::findContours(input, contours_, hierarchy_, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    std::vector< std::vector<cv::Point> >::iterator it;

    it = contours_.begin(); 
    for( ; it != contours_.end(); ){
      if(contourArea(*it) <= noise_size_) {
        it = contours_.erase(it);
        continue;
      }
      cv::Moments mu1 = moments(*it, false);
      cv::Point2f mc1 = cv::Point2f(mu1.m10/mu1.m00 , mu1.m01/mu1.m00);
      std::vector< std::vector<cv::Point> >::iterator itt = it+1;
      for( ; itt != contours_.end(); ){
        cv::Moments mu2 = moments(*itt, false);
        cv::Point2f mc2 = cv::Point2f(mu2.m10/mu2.m00 , mu2.m01/mu2.m00);
        double dist = sqrt((mc1.x - mc2.x)*(mc1.x - mc2.x)+
                           (mc1.y - mc2.y)*(mc1.y - mc2.y));
        if(dist <= min_dist_){
          mergeContours(*itt, *it);
          itt = contours_.erase(itt);
        }
        else ++itt;  
      }
      ++it;
    }

    convex_contours_.resize(contours_.size());
    poly_contours_.resize(contours_.size());
    for(unsigned int i = 0; i < contours_.size(); ++i){
      cv::convexHull(cv::Mat(contours_[i]), 
        convex_contours_[i], false);
      cv::approxPolyDP(cv::Mat(contours_[i]), 
        poly_contours_[i], epsilon_, close_);
    }

    //Filte out small contours and on border contours
    std::vector< std::vector<cv::Point> >::iterator itc = 
      convex_contours_.begin();
    for( ;itc != convex_contours_.end();){
      if( contourArea(*itc) <= min_size_ || onBorder(*itc, img_width_, img_height_ ) ){
        //NODELET_INFO("min Area:%lf",contourArea(*it));
        itc = convex_contours_.erase(itc);
      }
      else ++itc;
    }

    std::vector< std::vector<cv::Point> >::iterator itp = 
      poly_contours_.begin();
    for( ;itp != poly_contours_.end();){
      if( contourArea(*itp) <= min_size_ || onBorder(*itp, img_width_, img_height_ ) ){
        //NODELET_INFO("min Area:%lf",contourArea(*it));
        itp = poly_contours_.erase(itp);
      }
      else ++itp;
    }
    //NODELET_INFO("%lu,%lu,%lu",
    //  contours_.size(), convex_contours_.size(),poly_contours_.size());
    pubResult(image_msg->header);  
  }
}
  
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::ContourFinder, nodelet::Nodelet);
