#include "aricc_2d_vision/shapematch.h"


  struct Reference{
  std::string name;
};

  std::vector<Reference> reference_list_;


namespace aricc_2d_vision{

  //load these parameters on initialisation(?)
  void ShapeMatch::onInit(){
    ConnectionBasedNodelet::onInit();
    pnh_->param("debug", debug_, false);
    /*if(debug_){
      pub_debug_image_ = advertise<sensor_msgs::Image>(*pnh_, "debug", 1);
    }*/
    loadReferences();
  }

  //to get the following inputs
  void ShapeMatch::subscribe(){
    sub_contour_.subscribe(*pnh_,"input/contour", 1);
    sub_rgb_image_.subscribe(*pnh_,"input/rgb_image", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproxSyncPolicy> >(100);
      async_->connectInput(sub_contour_, sub_rgb_image_);
      async_->registerCallback(boost::bind(&ShapeMatch::execute, this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_contour_, sub_rgb_image_);
      sync_->registerCallback(boost::bind(&ShapeMatch::execute, this, _1, _2));
    }
  }

  //unsubscribe from the above inputs
  void ShapeMatch::unsubscribe(){
    sub_contour_.unsubscribe();
    sub_rgb_image_.unsubscribe();
  }
  
  //parameters that can be configured in the window?
  void ShapeMatch::configCallback(
    Config &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    debug_ = config.debug;
  }
  
  /*void ShapeMatch::pubDebug( cv::Mat& src, std_msgs::Header header ){
      

      pub_debug_image_.publish(cv_bridge::CvImage(
                               header,
                               sensor_msgs::image_encodings::BGR8,
                               src).toImageMsg());
      
      pub_debug_image_.publish(cv_bridge::CvImage(
                               header,
                               sensor_msgs::image_encodings::MONO8,
                               src).toImageMsg());
  }*/

  /*void ShapeMatch::ShapeMatch(std::vector<cv::Point> contour){

  }*/ 

  bool ShapeMatch::loadReferences(){

    reference_list_.clear();
    //Loading references from config files
    ROS_INFO("----------");
    ROS_INFO("Loading references ...");
    XmlRpc::XmlRpcValue reference_list;

    if( pnh_->getParam("references", reference_list) ){
      ROS_INFO("Found reference list, size:%d",reference_list.size());
      ROS_ASSERT(reference_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      if( reference_list.getType() == XmlRpc::XmlRpcValue::TypeArray ){
        for( size_t i = 0; i < reference_list.size(); ++i){
          std::string name = std::string(reference_list[i]["name"]);
          std::string file = std::string(reference_list[i]["file"]);
          //NODELET_INFO("%s, %s", name.c_str(), file.c_str());
          /*Reference rf;
          //wrong variable type for contours?
          rf.contours = cv::findContours(input, contours_, hierarchy_, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

          reference_list_.push_back(rf);*/
        }
        ROS_INFO("Loaded %lu references",reference_list_.size());
        return true;
      }
    }
    else ROS_ERROR("Couldn`t load references");
    ROS_INFO("----------");
    return false;

  }

  void ShapeMatch::execute(    
    const aricc_vision_msgs::ContourArray::ConstPtr& msg,
    const sensor_msgs::Image::ConstPtr& image_msg
){
  
  /*if (debug_){

      cv::Scalar red = cv::Scalar( 0, 0, 255 );
      cv::Mat drawing = cv::Mat::zeros(cv::Size(img_width_, img_height_), CV_8UC3);
      if(draw_contours_){
        for(size_t i = 0; i < contours_.size(); ++i){
          cv::drawContours(drawing, contours_, i, red, 2, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        }
      }


  }*/

  }

}
