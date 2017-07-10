#include <aricc_2d_vision/check_board_detector.h>

namespace aricc_2d_vision{
  void CheckBoardDetector::onInit(){
    ConnectionBasedNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("debug", debug_, true);
    pnh_->param("verbose", verbose_, false);
    pnh_->param("invert_color", invert_color_, false);
    pnh_->param("use_p", use_P_, false);
    pnh_->param("board_type", board_type_, std::string(""));
    pnh_->param("max_borad", max_board_, 1);
    pnh_->param("columns", columns_, 1);
    pnh_->param("rows", rows_, 1);
    pnh_->param("rect_size_cols", rect_size_x_, 1.0);
    pnh_->param("rect_size_rows", rect_size_y_, 1.0);
    pnh_->param("border_thresh", border_thresh_, 30);
    
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &CheckBoardDetector::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    //Initial grid3d
    grid3d_.clear();
    grid3d_.resize(columns_*rows_);
    unsigned int index = 0;
    if(board_type_ == "chess" || board_type_ == "circle"){
      for(unsigned int y = 0; y < rows_; ++y){
        for(unsigned int x = 0; x < columns_; ++x){
          grid3d_[index++] = cv::Point3f(x*rect_size_x_, y*rect_size_y_, 0);
        }
      }
    }

    pub_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output", 1);
    if(debug_)
      pub_debug_ = advertise<sensor_msgs::Image>(*pnh_, "output/debug", 1);
    
  }

  void CheckBoardDetector::subscribe(){
    sub_camera_.subscribe(*pnh_, "input", 1);
    sub_camera_info_.subscribe(*pnh_,"input/camera_info", 1);
  
    if(approximate_sync_){
      async_ = boost::make_shared<message_filters::Synchronizer<ApproxSyncPolicy> >(100);
      async_->connectInput(sub_camera_, sub_camera_info_);
      async_->registerCallback(boost::bind(&CheckBoardDetector::execute, this, _1, _2));  
    }
    else{
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_camera_, sub_camera_info_);
      sync_->registerCallback(boost::bind(&CheckBoardDetector::execute, this, _1, _2));
    }
    
  }

  void CheckBoardDetector::unsubscribe(){
    sub_camera_.unsubscribe();
    sub_camera_info_.unsubscribe();
  }

  void CheckBoardDetector::configCallback(
    Config &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    debug_ = config.debug;
    invert_color_ = config.invert_color;
    use_P_ = config.use_P;
    max_board_ = config.max_board;
    columns_ = config.columns;
    rows_ = config.rows;
    border_thresh_ = config.border_thresh;
    rect_size_x_ = config.rect_size_cols;
    rect_size_y_ = config.rect_size_rows;
    board_type_ = config.board_type;
  }

  void CheckBoardDetector::execute(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::CameraInfo::ConstPtr& info_msg){
    //NODELET_INFO("Executing ...");
    boost::mutex::scoped_lock lock(mutex_);
    
    bool found = detect(image_msg, info_msg);
    if(!found) return;

    if(debug_) displayResult(image_msg, info_msg);
    publishResult(image_msg->header);
  }

  bool CheckBoardDetector::detect(
    const sensor_msgs::Image::ConstPtr& image_msg, 
    const sensor_msgs::CameraInfo::ConstPtr& info_msg){

    image_geometry::PinholeCameraModel model;
    sensor_msgs::CameraInfo cam_info(*info_msg);

    if( cam_info.distortion_model.empty() ){
      cam_info.distortion_model = "plumb_bob";
      cam_info.D.resize(5, 0);
    }
    if(use_P_){
      for(unsigned int i = 0; i < cam_info.D.size(); ++i){
        cam_info.D[i] = 0.0;
      }
    }
    // check all the value of R is zero or not
    // if zero, normalzie it
    if (use_P_ || std::equal(cam_info.R.begin() + 1, cam_info.R.end(), cam_info.R.begin())) {
      cam_info.R[0] = 1.0;
      cam_info.R[4] = 1.0;
      cam_info.R[8] = 1.0;
    }

    // check all the value of K is zero or not
    // if zero, copy all the value from P
    if (use_P_ || std::equal(cam_info.K.begin() + 1, cam_info.K.end(), cam_info.K.begin())) {
      cam_info.K[0] = cam_info.P[0];
      cam_info.K[1] = cam_info.P[1];
      cam_info.K[2] = cam_info.P[2];
      cam_info.K[3] = cam_info.P[4];
      cam_info.K[4] = cam_info.P[5];
      cam_info.K[5] = cam_info.P[6];
      cam_info.K[6] = cam_info.P[8];
      cam_info.K[7] = cam_info.P[9];
      cam_info.K[8] = cam_info.P[10];
    }
    model.fromCameraInfo(cam_info);

    cv::Mat input;
    try{
      //Input is the color image
      if(image_msg->encoding == sensor_msgs::image_encodings::BGR8 ||
         image_msg->encoding == sensor_msgs::image_encodings::RGB8){
        input = cv_bridge::toCvCopy(
          image_msg, sensor_msgs::image_encodings::MONO8)->image;
      }
      else if(image_msg->encoding == sensor_msgs::image_encodings::MONO8){
        input = cv_bridge::toCvShare(image_msg,
                                   image_msg->encoding)->image;;
      }
      else {
        NODELET_ERROR("Wrong image encoding, only support RGB8 and MONO8");
        return false;
      }
    } 
    catch(cv_bridge::Exception &e){
      NODELET_ERROR("Failed to get image from camera %s", e.what());
      return false;
    }
    if(invert_color_){
      input = cv::Mat((input + 0.0) * 1.0 / 1.0) * 1.0;
      cv::Mat temp;
      cv::bitwise_not(input, temp);
      input = temp;
    }

    //Trying to find all checkboards
    int board_cnt = 0;
    cv::Size pattern_size(columns_, rows_);

    bool found = false;
    if( board_type_ == "chess" ){
        //This will be filled by the detected corners
        std::vector<cv::Point2f> corners;
        corners.clear();

        found = cv::findChessboardCorners(input, pattern_size, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
        + cv::CALIB_CB_FAST_CHECK);
        /*
        NODELET_INFO("Input: %lu, Found: %d, corner:%lu, size: %d, %d", 
          input.total(),found, corners.size(), pattern_size.width, 
          pattern_size.height);
        cv::drawChessboardCorners(input, pattern_size, cv::Mat(corners), found);
        std_msgs::Header header = image_msg->header;
        header.stamp = ros::Time::now();
        pub_debug_.publish( cv_bridge::CvImage(
          header,
          sensor_msgs::image_encodings::MONO8,
          input).toImageMsg());
        */

        if(!found || corners.size() != grid3d_.size()) return false;

        //Remove any corners that are so close to border
        for(unsigned int i = 0; i < corners.size(); ++i){
          unsigned int x = corners[i].x;
          unsigned int y = corners[i].y;
          if( x < border_thresh_ || x > (input.cols - border_thresh_) ||
              y < border_thresh_ || y > (input.rows - border_thresh_) ){
            return false;
          }
        }
        
        //Calculate the window size for refining the corner locations
        cv::Point upper_left, lower_right;
        upper_left.x = lower_right.x = corners[0].x;
        upper_left.y = lower_right.y = corners[0].y;
        for(unsigned  i = 1; i < corners.size(); ++i){
          if( upper_left.x > corners[i].x ) 
            upper_left.x = corners[i].x;
          if( upper_left.y > corners[i].y ) 
            upper_left.y = corners[i].y;
          if( lower_right.x < corners[i].x ) 
            lower_right.x = corners[i].x;
          if( lower_right.y < corners[i].y ) 
            lower_right.y = corners[i].y;
        }
        double step_size =
          (double)( ((upper_left.x - lower_right.x) * 
          (upper_left.x - lower_right.x)) +
          ((upper_left.y - lower_right.y) * 
          (upper_left.y - lower_right.y)) ) /
          ( ((pattern_size.width - 1) * (pattern_size.width - 1)) +
          ( (pattern_size.height - 1) * (pattern_size.height - 1)) );
        unsigned int win_size = (unsigned int)(0.5*sqrt(step_size) + 0.5);
        cv::cornerSubPix(input, corners, 
          cv::Size(win_size, win_size), cv::Size(-1, -1),
          cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        //Get tf 6D
        tf::Transform tf; 
        tf = findTransformation(corners, grid3d_, model);
        CheckBoard cb;
        cb.corners = corners;
        cb.tf = tf;
        detected_boards_ = cb;
        
        /*
        //Debuging
        cv::drawChessboardCorners(input, pattern_size, cv::Mat(corners), found);
        std_msgs::Header header = image_msg->header;
        header.stamp = ros::Time::now();
        pub_debug_.publish( cv_bridge::CvImage(
          header,
          sensor_msgs::image_encodings::MONO8,
          input).toImageMsg());
        */
    }

    else if( board_type_ == "circle"){
        //This will be filled by the detected corners
        std::vector<cv::Point2f> centers;
        centers.clear();
        found = cv::findCirclesGrid( input, pattern_size, centers,
        cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING); 
    }

    else if(board_type_ == "qrcode"){
      //TODO: Add QR CODE detection
    }
    return found;
  }

  tf::Transform CheckBoardDetector::findTransformation(
    const std::vector<cv::Point2f> &img_points,
    const std::vector<cv::Point3f> &grid_points,
    const image_geometry::PinholeCameraModel model){
    cv::Mat rot_mat, trans_mat;
    cv::solvePnP(grid_points, img_points, model.intrinsicMatrix(),
      model.distortionCoeffs(), rot_mat, trans_mat, false);
    double trans[3] = {0};
    double rot[3] = {0};

    //Change from OpenCV to ROS coordinate
    trans[0] = trans_mat.at<double>(2);
    trans[1] = -1.0*trans_mat.at<double>(0);
    trans[2] = -1.0*trans_mat.at<double>(1);
   
    rot[0] = rot_mat.at<double>(2);
    rot[1] = -1.0*rot_mat.at<double>(0);
    rot[2] = -1.0*rot_mat.at<double>(1);
    
    /*
    for(unsigned int i = 0; i < 3; ++i){
      rot[i] = rot_mat.at<double>(i);
      trans[i] = trans_mat.at<double>(i);
    }

    tf::Matrix3x3 tf_rot_check;
    tf::Matrix3x3 tf_rot_ros;
    tf_rot_check.setRPY(rot[0], rot[1], rot[2]);
    tf_rot_ros.setRPY(-M_PI_2, 0, -M_PI_2);
    */

    //NODELET_INFO("OpenCV:");
    //NODELET_INFO("Trans:%lf,%lf,%lf",trans[0], trans[1], trans[2]);
    //NODELET_INFO("Rot:%lf,%lf,%lf",rot[0],rot[1],rot[2]);

    /*
    tf::Vector3 tf_trans(trans[0], trans[1], trans[2]);
    tf::Matrix3x3 tf_rot;
    //tf_rot.setRPY(rot[0],rot[1],rot[2]);
    tf_rot = tf_rot_check;
    tf::Transform tf_check(tf_rot, tf_trans);
    tf::Transform tf_ros( tf_rot_ros, tf::Vector3(0,0,0));
    tf_check = tf_ros * tf_check;
    */
    double offset_y = -1.0*rect_size_x_*(columns_-1)/2.0;
    double offset_z = -1.0*rect_size_y_*(rows_-1)/2.0;
    tf::Vector3 trans_offset(0.0, offset_y, offset_z);
    tf::Matrix3x3 rot_offset;
    rot_offset.setRPY(0.0,0.0,0.0);
    tf::Transform tf_offset(rot_offset, trans_offset);

    tf::Vector3 tf_trans(trans[0], trans[1], trans[2]);
    tf::Matrix3x3 tf_rot;
    tf_rot.setRPY(rot[0],rot[1],rot[2]);
    tf::Transform tf_check(tf_rot, tf_trans);

    tf_check = tf_check * tf_offset;

    //static tf::TransformBroadcaster br;
    //br.sendTransform(tf::StampedTransform(tf_check, ros::Time::now(), 
    //  "camera", "checkboard"));
    /*
    tf::Vector3 pos  = tf_check.getOrigin();
    tf::Matrix3x3 ort = tf_check.getBasis();
    double roll, pitch, yaw;
    ort.getRPY(roll, pitch, yaw);

    roll = aricc_utils::smallAngle(roll) ;
    pitch = aricc_utils::smallAngle(pitch);
    yaw = aricc_utils::smallAngle(yaw) ;
    */
    /*
    NODELET_INFO("ROS:");
    NODELET_INFO("Trans:%lf,%lf,%lf",pos.getX(), pos.getY(),pos.getZ());
    NODELET_INFO("Ort:%lf,%lf,%lf",roll, pitch, yaw);
    */    

    return tf_check;
  }

  void CheckBoardDetector::displayResult(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::CameraInfo::ConstPtr& info_msg){ 
    //NODELET_INFO("Displaying result ...");
    cv::Mat input;
    try{
      //Input is the color image
      if(image_msg->encoding == sensor_msgs::image_encodings::BGR8 ||
         image_msg->encoding == sensor_msgs::image_encodings::RGB8 ||
         image_msg->encoding == sensor_msgs::image_encodings::MONO8){
        input = cv_bridge::toCvShare(image_msg,
                                   image_msg->encoding)->image;;
      }
      else {
        NODELET_ERROR("Wrong image encoding, only support RGB8 and MONO8");
        return;
      }
    }
    catch(cv_bridge::Exception &e){
      NODELET_ERROR("Failed to get image from camera %s", e.what());
      return;
    }

    CheckBoard cb = detected_boards_;
    std::string cb_link = board_type_;
      //NODELET_INFO("%s", cb_link.str().c_str());
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(cb.tf, ros::Time::now(), 
      image_msg->header.frame_id, cb_link.c_str()));

    //Draw checkboard
    cv::Size pattern_size(columns_, rows_); 
    cv::drawChessboardCorners(input, pattern_size, cv::Mat(cb.corners), true);
      
    //Draw TF on image
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(info_msg);
    tf::Vector3 pos  = cb.tf.getOrigin();
    tf::Matrix3x3 ort = cb.tf.getBasis();
    double roll, pitch, yaw;
    ort.getRPY(roll, pitch, yaw);

    roll = aricc_utils::smallAngle(roll) ;
    pitch = aricc_utils::smallAngle(pitch);
    yaw = aricc_utils::smallAngle(yaw);
    
    if(verbose_){
      NODELET_INFO("Ort(in degree):%lf, %lf, %lf", aricc_utils::rad2Deg(roll),
        aricc_utils::rad2Deg(pitch), aricc_utils::rad2Deg(yaw));
      NODELET_INFO("Pos:%lf, %lf, %lf", pos.getX(), pos.getY(), pos.getZ());
    }
    //cv::Point3d pt_3d(pt.x(), pt.y(), pt.z());
    cv::Point3d pt_3d(-pos.getY(), -pos.getZ(), pos.getX());
    cv::Point2d pt_2d;
    pt_2d = cam_model.project3dToPixel(pt_3d);
    //NODELET_INFO("ROS(%lf, %lf, %lf)", pt.x(), pt.y(), pt.z());
    //NODELET_INFO("OPENCV(%lf, %lf)", pt_2d.x, pt_2d.y);
     
    static const int rad = 3;
    cv::circle(input, pt_2d, rad, cv::Scalar(255,0,0), -1);
    cv::Size text_size;
    CvFont font;
    int baseline = 0;
    text_size = cv::getTextSize(cb_link.c_str(), cv::FONT_HERSHEY_SIMPLEX,
                          1, 1, &baseline);
    cv::Point origin = cvPoint(pt_2d.x - text_size.width / 2,
                             pt_2d.y - rad - baseline - 3);
    cv::putText(input, cb_link.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 1, 
      cv::Scalar(255,0,0));
    
    //Draw center point of image
    double z = pos.getX();
    pt_3d = cv::Point3d(0.0, 0.0, z);
    pt_2d = cam_model.project3dToPixel(pt_3d);
    cv::circle(input, pt_2d, 2, cv::Scalar(0,255,0), -1);    
    cv::circle(input, pt_2d, 6, cv::Scalar(0,255,0), 1);

    cv::Point2d pt_2d_1, pt_2d_2;
    pt_3d = cv::Point3d(rect_size_x_/2.0, 0.0, z);
    pt_2d_1 = cam_model.project3dToPixel(pt_3d);    
    pt_3d = cv::Point3d(-rect_size_x_/2.0, 0.0, z);
    pt_2d_2 = cam_model.project3dToPixel(pt_3d);    
    cv::line(input, pt_2d_1, pt_2d_2, cv::Scalar(0,255,0));

    pt_3d = cv::Point3d(0.0, rect_size_y_/2.0, z);
    pt_2d_1 = cam_model.project3dToPixel(pt_3d);    
    pt_3d = cv::Point3d(0.0, -rect_size_y_/2.0, z);
    pt_2d_2 = cam_model.project3dToPixel(pt_3d);    
    cv::line(input, pt_2d_1, pt_2d_2, cv::Scalar(0,255,0));

 
    std_msgs::Header header = image_msg->header;
    header.stamp = ros::Time::now();
    pub_debug_.publish( cv_bridge::CvImage(
        header,
        image_msg->encoding,
        input).toImageMsg());
  }

  void CheckBoardDetector::publishResult(std_msgs::Header header){
    geometry_msgs::PoseStamped pose;
    CheckBoard cb = detected_boards_;

    tf::Vector3 pos  = cb.tf.getOrigin();
    tf::Matrix3x3 ort = cb.tf.getBasis();
    double roll, pitch, yaw;
    ort.getRPY(roll, pitch, yaw);

    roll = aricc_utils::smallAngle(roll) ;
    pitch = aricc_utils::smallAngle(pitch);
    yaw = aricc_utils::smallAngle(yaw);

    geometry_msgs::Quaternion q;
    aricc_utils::euler2Quat(roll, pitch, yaw, q);

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = header.frame_id;
    pose.pose.position.x = pos.getX();
    pose.pose.position.y = pos.getY();
    pose.pose.position.z = pos.getZ();
    pose.pose.orientation = q;

    pub_.publish(pose);

 
  }
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_2d_vision::CheckBoardDetector, nodelet::Nodelet);
