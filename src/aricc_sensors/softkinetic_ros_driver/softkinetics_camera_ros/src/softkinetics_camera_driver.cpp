/*!
 *****************************************************************
 *   Copyright (c) 2015
 *   ARICC
 *
 *****************************************************************
 *
 *   ROS package name: softkinetic_camera_nodelet
 *
 *   Author: Ian Wang, email: ariccsg@gmail.com
 *
 *   Date of creation: March 2015
 *
 *****************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************/
#include <softkinetics_camera_ros/softkinetics_camera_driver.h>


  SoftKineticsCamera::SoftKineticsCamera():initialized_(false){
    pnh_ = ros::NodeHandle("~");
    //Initial Device
    g_context_ = Context::create("softkinetic");
    //g_context_ = DepthSense::Context::createStandalone();
    // Get the list of currently connected devices
    devices_ = g_context_.getDevices();
    ROS_INFO("Number of Devices found: %lu",devices_.size());
    if( devices_.size() == 0 ){ 
      ROS_ERROR("No devices found!");
      return;
    }
    
    pnh_.param<std::string>("camera_link", base_link_, "softkinetic_link");
    pnh_.param("color_enabled", color_enabled_, false);
    pnh_.param("depth_enabled", depth_enabled_, false);
    pnh_.param("cloud_enabled", cloud_enabled_, false);
    pnh_.param("audio_enabled", audio_enabled_, false);
    pnh_.param("confidence_threshold", confidence_threshold_, 50);

    std::string depth_mode_str;
    pnh_.param<std::string>("depth_mode_",   depth_mode_str, "close");
    if ( depth_mode_str == "long" )
      depth_mode_ = DepthNode::CAMERA_MODE_LONG_RANGE;
    else
      depth_mode_ = DepthNode::CAMERA_MODE_CLOSE_MODE;
    
    std::string depth_frame_format_str;
    pnh_.param<std::string>("depth_frame_format", depth_frame_format_str, "QVGA");
    if ( depth_frame_format_str == "QQVGA" )
      depth_frame_format_ = FRAME_FORMAT_QQVGA;
    else if ( depth_frame_format_str == "QVGA" )
      depth_frame_format_ = FRAME_FORMAT_QVGA;
    else
      depth_frame_format_ = FRAME_FORMAT_VGA;    

    pnh_.param("depth_frame_rate", depth_frame_rate_, 25);
    

    std::string color_compression_str;
    pnh_.param<std::string>("color_compression", color_compression_str, "MJPEG");
    if ( color_compression_str == "YUY2" )
      color_compression_ = COMPRESSION_TYPE_YUY2;
    else
      color_compression_ = COMPRESSION_TYPE_MJPEG;

    std::string color_frame_format_str;
    pnh_.param<std::string>("color_frame_format", color_frame_format_str, "VGA");
    if ( color_frame_format_str == "QQVGA" )
      color_frame_format_ = FRAME_FORMAT_QQVGA;
    else if ( color_frame_format_str == "QVGA" )
      color_frame_format_ = FRAME_FORMAT_QVGA;
    else if ( color_frame_format_str == "VGA" )
      color_frame_format_ = FRAME_FORMAT_VGA;
    else if ( color_frame_format_str == "NHD" )
      color_frame_format_ = FRAME_FORMAT_NHD;
    else
      color_frame_format_ = FRAME_FORMAT_WXGA_H;

    pnh_.param("color_frame_rate", color_frame_rate_, 25);
    
    transforms_ = new tf::TransformBroadcaster();

    if(cloud_enabled_){
      ROS_INFO("Cloud stream enabled");
      color_enabled_ = true;
      depth_enabled_ = true;
      pub_cloud_ = pnh_.advertise<sensor_msgs::PointCloud2>( "cloud", 1);
    }
    if(color_enabled_){
      ROS_INFO("Color stream enabled");
      it_rgb_.reset(new image_transport::ImageTransport(pnh_));
      pub_rgb_ = it_rgb_->advertiseCamera("rgb/image", 1, false); 
    }
    if(depth_enabled_){ 
      ROS_INFO("Depth stream enabled");
      it_depth_.reset(new image_transport::ImageTransport(pnh_));
      pub_depth_ = it_depth_->advertiseCamera("depth/image", 1, false); 
    } 
    
    g_context_.deviceAddedEvent().connect(this,&SoftKineticsCamera::onDeviceConnected);
    g_context_.deviceRemovedEvent().connect(this,&SoftKineticsCamera::onDeviceDisconnected);
    // In case there are several devices, 
    // index of camera to start ought to come as an argument. 
    //By default, index 0 is taken:
    if(devices_.size() > 1) ROS_WARN("This driver only supports ONE device, the first device will be taken");
    g_bDeviceFound_ = true;
    devices_[0].nodeAddedEvent().connect(this,&SoftKineticsCamera::onNodeConnected);
    devices_[0].nodeRemovedEvent().connect(this, &SoftKineticsCamera::onNodeDisconnected);
    std::vector<Node> nodes = devices_[0].getNodes();
    for(unsigned int i = 0; i < nodes.size(); ++i) configureNode(nodes[i]);
    camera_params_ = devices_[0].getStereoCameraParameters();  
    initialized_ = true;
  }

  SoftKineticsCamera::~SoftKineticsCamera(){
    shutdown();
  }

  bool SoftKineticsCamera::start(){
    if(initialized_){
      g_context_.startNodes();
      g_context_.run();
    }
  }

  void SoftKineticsCamera::shutdown(){
    if(!initialized_) return;
    ROS_INFO("Shutting down device ...");
    //Close out all nodes
    if (g_cnode_.isSet()) g_context_.unregisterNode(g_cnode_);
    if (g_dnode_.isSet()) g_context_.unregisterNode(g_dnode_);
    if (g_anode_.isSet()) g_context_.unregisterNode(g_anode_);

    if (g_pProjHelper_) delete g_pProjHelper_;
    devices_[0].nodeAddedEvent().disconnect(this,&SoftKineticsCamera::onNodeConnected);
    devices_[0].nodeRemovedEvent().disconnect(this, &SoftKineticsCamera::onNodeDisconnected);
    g_context_.deviceAddedEvent().disconnect(this,&SoftKineticsCamera::onDeviceConnected);
    g_context_.deviceRemovedEvent().disconnect(this,&SoftKineticsCamera::onDeviceDisconnected);

    g_context_.stopNodes();
    g_context_.quit();
    initialized_ = false; 
  }
  
  double SoftKineticsCamera::timeStampToRosTime(uint64_t timestamp){
    timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    double monotonic_time = time.tv_sec  + time.tv_nsec * 1.0e-9;
    double ros_time = ros::Time::now().toSec();
    return ros_time - (monotonic_time - 1.0e-6 * timestamp);
  }

  // New audio sample event handler
  void SoftKineticsCamera::onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data){
  }

  template <typename T>
  T SoftKineticsCamera::clip(const T& n, const T& lower, const T& upper) {
    return std::max(lower, std::min(n, upper));
  }

  void SoftKineticsCamera::YUV2RGB( DepthSense::Pointer<unsigned char> yuvData, 
                unsigned char* rgbData, int width, int height){
    for(int i = 0, j = 0; i < width * height * 3; i+=6, j+=4){
      int y1  = yuvData[j+0];
      int u   = yuvData[j+1];
      int y2  = yuvData[j+2];
      int v   = yuvData[j+3];

      int C1 = y1 - 16;
      int C2 = y2 - 16;
      int D = u - 128;
      int E = v - 128;

      rgbData[i+0] = (unsigned char)clip((298*C1+516*D+128)/256, 0, 255 );
      rgbData[i+1] = (unsigned char)clip((298*C1-100*D-208*E+128)/256,0,255);
      rgbData[i+2] = (unsigned char)clip((298*C1+409*E+128)/256, 0, 255);
      rgbData[i+3] = (unsigned char)clip((298*C2+516*D+128)/256, 0, 255);
      rgbData[i+4] = (unsigned char)clip((298*C2-100*D-208*E+128)/256, 0, 255);
      rgbData[i+5] = (unsigned char)clip((298*C2+409*E+128)/256, 0, 255);
    }
  }

  // New color sample event handler
  void SoftKineticsCamera::onNewColorSample(ColorNode node, 
    ColorNode::NewSampleReceivedData data){
    int sub_rgb = pub_rgb_.getNumSubscribers();
    if(sub_rgb > 0 || cloud_enabled_){
      rgb_image_msg_.header.frame_id = base_link_+ "/rgb_optical_frame";
      rgb_image_msg_.header.stamp =  ros::Time(timeStampToRosTime(data.timeOfCapture));
      int32_t rgb_width, rgb_height;
      FrameFormat_toResolution(data.captureConfiguration.frameFormat,
        &rgb_width,&rgb_height);
      //ROS_INFO("%d,%d",w,h);
      rgb_image_msg_.width = rgb_width;///2;
      rgb_image_msg_.height = rgb_height;///2;
      rgb_image_msg_.step = rgb_width*3;
      if(data.captureConfiguration.compression == COMPRESSION_TYPE_YUY2){ 
        rgb_image_msg_.data.resize(rgb_width*rgb_height*3);
        rgb_image_msg_.encoding = sensor_msgs::image_encodings::BGR8;
        //std::memcpy(image.data.data(), data.colorMap, data.colorMap.size());
        YUV2RGB(data.colorMap, rgb_image_msg_.data.data(), 
          rgb_image_msg_.width, rgb_image_msg_.height);
      }
      else {
        rgb_image_msg_.encoding = sensor_msgs::image_encodings::BGR8;
        rgb_image_msg_.data.resize(rgb_width*rgb_height*3);
        std::memcpy(rgb_image_msg_.data.data(), data.colorMap, data.colorMap.size());
      }
      //camera_params_ = data.stereoCameraParameters;  
      const IntrinsicParameters& params = camera_params_.colorIntrinsics;
      rgb_camInfo_msg_.header.frame_id = base_link_+ "/rgb_optical_frame";
      rgb_camInfo_msg_.header.stamp    = rgb_image_msg_.header.stamp;
      rgb_camInfo_msg_.width           = rgb_image_msg_.width;
      rgb_camInfo_msg_.height          = rgb_image_msg_.height;
      rgb_camInfo_msg_.distortion_model = "plumb_bob";

      rgb_camInfo_msg_.D.resize(5);
      rgb_camInfo_msg_.D[0] = params.k1;
      rgb_camInfo_msg_.D[1] = params.k2;
      rgb_camInfo_msg_.D[4] = params.k3;

      rgb_camInfo_msg_.K[0] = params.fx;
      rgb_camInfo_msg_.K[2] = params.cx;
      rgb_camInfo_msg_.K[4] = params.fy;
      rgb_camInfo_msg_.K[5] = params.cy;
      rgb_camInfo_msg_.K[8] = 1.0f;

      rgb_camInfo_msg_.R[0] = 1;
      rgb_camInfo_msg_.R[4] = 1;
      rgb_camInfo_msg_.R[7] = 1;

      rgb_camInfo_msg_.P[0] = params.fx;
      rgb_camInfo_msg_.P[2] = params.cx;
      rgb_camInfo_msg_.P[5] = params.fy;
      rgb_camInfo_msg_.P[6] = params.cy;
      rgb_camInfo_msg_.P[10] = 1;
      if(sub_rgb > 0){
        //Broadcast TF 
        tf::StampedTransform color_transform;
        color_transform.frame_id_ = base_link_;
        color_transform.child_frame_id_ = base_link_ + "/rgb_optical_frame";
        color_transform.stamp_ = ros::Time(timeStampToRosTime(data.timeOfCapture));;
        color_transform.setOrigin(tf::Vector3(0, 0, 0));
        color_transform.setBasis(tf::Matrix3x3::getIdentity());
        transforms_->sendTransform(color_transform);
        pub_rgb_.publish(rgb_image_msg_, rgb_camInfo_msg_);
      }
    }
  }

  // New depth sample event
  void SoftKineticsCamera::onNewDepthSample(DepthNode node, 
    DepthNode::NewSampleReceivedData data){
    
    int sub_depth = pub_depth_.getNumSubscribers();
    if(sub_depth > 0 || cloud_enabled_){ 
      //fill in the depth image message header
      depth_image_msg_.header.frame_id = base_link_ + "/depth_optical_frame";
      depth_image_msg_.header.stamp = 
        ros::Time(timeStampToRosTime(data.timeOfCapture));
      int32_t depth_width, depth_height;
      FrameFormat_toResolution(data.captureConfiguration.frameFormat,
        &depth_width,&depth_height);
      depth_image_msg_.width = depth_width;
      depth_image_msg_.height = depth_height;
      depth_image_msg_.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      depth_image_msg_.is_bigendian = 0;
      depth_image_msg_.step = sizeof(float)*depth_width;
      std::size_t data_size = depth_width * depth_height;
      depth_image_msg_.data.resize(data_size*sizeof(float));
      std::memcpy( depth_image_msg_.data.data(), data.depthMapFloatingPoint, 
        depth_image_msg_.data.size());
    
      const IntrinsicParameters& params = camera_params_.depthIntrinsics;

      depth_camInfo_msg_.header.frame_id = base_link_ + "/depth_optical_frame";
      depth_camInfo_msg_.header.stamp = depth_image_msg_.header.stamp;
      depth_camInfo_msg_.width = depth_width;
      depth_camInfo_msg_.height = depth_height;
      depth_camInfo_msg_.distortion_model = "plumb_bob";
    
      depth_camInfo_msg_.D.resize(5);
      depth_camInfo_msg_.D[0] = params.k1;
      depth_camInfo_msg_.D[1] = params.k2;
      depth_camInfo_msg_.D[4] = params.k3;

      depth_camInfo_msg_.K[0] = params.fx;
      depth_camInfo_msg_.K[2] = params.cx;
      depth_camInfo_msg_.K[4] = params.fy;
      depth_camInfo_msg_.K[5] = params.cy;
      depth_camInfo_msg_.K[8] = 1.0f;

      depth_camInfo_msg_.R[0] = 1;
      depth_camInfo_msg_.R[4] = 1;
      depth_camInfo_msg_.R[7] = 1;

      depth_camInfo_msg_.P[0] = params.fx;
      depth_camInfo_msg_.P[2] = params.cx;
      depth_camInfo_msg_.P[5] = params.fy;
      depth_camInfo_msg_.P[6] = params.cy;
      depth_camInfo_msg_.P[10] = 1;
      
      if(sub_depth > 0){
        tf::StampedTransform depth_transform;
        depth_transform.frame_id_ = base_link_;
        depth_transform.child_frame_id_ = base_link_ + "/depth_optical_frame";
        depth_transform.stamp_ = depth_image_msg_.header.stamp;
        depth_transform.setOrigin(tf::Vector3(camera_params_.extrinsics.t1, camera_params_.extrinsics.t2, camera_params_.extrinsics.t3));
        depth_transform.setBasis(tf::Matrix3x3(camera_params_.extrinsics.r11, camera_params_.extrinsics.r12, camera_params_.extrinsics.r13,
                                          camera_params_.extrinsics.r21, -camera_params_.extrinsics.r22, camera_params_.extrinsics.r23,
                                          camera_params_.extrinsics.r31, -camera_params_.extrinsics.r32, camera_params_.extrinsics.r33));
        transforms_->sendTransform(depth_transform);
        pub_depth_.publish(depth_image_msg_, depth_camInfo_msg_);
      }
    }

    int sub_cloud = pub_cloud_.getNumSubscribers();
    if(!cloud_enabled_ || sub_cloud == 0) return;   

    // Project some 3D points in the Color Frame
    if (!g_pProjHelper_){
        g_pProjHelper_ = new ProjectionHelper(data.stereoCameraParameters);
        camera_params_ = data.stereoCameraParameters;
    }
    else {
      if (camera_params_ != data.stereoCameraParameters){
        g_pProjHelper_->setStereoCameraParameters(data.stereoCameraParameters);
        camera_params_ = data.stereoCameraParameters;
      }
    }
    Vertex p3DPoints[1];
    Point2D p2DPoints[1];

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    int32_t cloud_width, cloud_height;
    FrameFormat_toResolution(data.captureConfiguration.frameFormat,&cloud_width,&cloud_height);
    current_cloud->height = cloud_height;
    current_cloud->width = cloud_width;
    current_cloud->is_dense = false;
    current_cloud->points.resize(cloud_height*cloud_width);
    uchar b, g, r;
    int count = -1;
    for(unsigned int i = 1;i < current_cloud->height ;++i){
      for(unsigned int j = 1;j < current_cloud->width ; ++j){
        count++;
        current_cloud->points[count].x = data.verticesFloatingPoint[count].x;
        current_cloud->points[count].y = -data.verticesFloatingPoint[count].y;
        if( data.verticesFloatingPoint[count].z == 32001 || 
            data.depthMapFloatingPoint[count] < 0.001 || 
            data.depthMapFloatingPoint[count] > 3){
          current_cloud->points[count].z = std::numeric_limits<float>::quiet_NaN();
        }
        else{
          current_cloud->points[count].z = data.verticesFloatingPoint[count].z;
        }
        //Saturated pixels on depthMapFloatingPoint have -1 value, 
        //but on openni are NaN
        if(data.depthMapFloatingPoint[count] < 0.001){
          *reinterpret_cast<float*>(&depth_image_msg_.data[count*sizeof(float)]) = std::numeric_limits<float>::quiet_NaN();
        }
        //Get mapping between depth map and color map, 
        //assuming we have a RGB image
        if(rgb_image_msg_.data.size() == 0){
          ROS_WARN("Color image is empty; pointcloud will be colorless");
          continue;
        }
        p3DPoints[0] = data.vertices[count];
	g_pProjHelper_->get2DCoordinates(p3DPoints, p2DPoints, 2, CAMERA_PLANE_COLOR);
        int x_pos = (int)p2DPoints[0].x;
        int y_pos = (int)p2DPoints[0].y;

        if( y_pos < 0 || y_pos > rgb_image_msg_.height || 
            x_pos < 0 || x_pos > rgb_image_msg_.width){
          b = 0;
          g = 0;
          r = 0;
        }
        else{
          b = rgb_image_msg_.data[(y_pos*rgb_image_msg_.width+x_pos)*3+0];
          g = rgb_image_msg_.data[(y_pos*rgb_image_msg_.width+x_pos)*3+1];
          r = rgb_image_msg_.data[(y_pos*rgb_image_msg_.width+x_pos)*3+2];
        }
        current_cloud->points[count].b = b;
        current_cloud->points[count].g = g;
        current_cloud->points[count].r = r;
      }
    }
    //convert current_cloud to PointCloud2 and publish
    tf::StampedTransform cloud_transform;
    cloud_transform.frame_id_ = base_link_;
    cloud_transform.child_frame_id_ = base_link_ + "/cloud_frame";
    cloud_transform.stamp_ = ros::Time(timeStampToRosTime(data.timeOfCapture));
    cloud_transform.setOrigin(tf::Vector3(camera_params_.extrinsics.t1, camera_params_.extrinsics.t2, camera_params_.extrinsics.t3));
    cloud_transform.setBasis(tf::Matrix3x3(camera_params_.extrinsics.r11, camera_params_.extrinsics.r12, camera_params_.extrinsics.r13,
                                          camera_params_.extrinsics.r21, -camera_params_.extrinsics.r22, camera_params_.extrinsics.r23,
                                          camera_params_.extrinsics.r31, -camera_params_.extrinsics.r32, camera_params_.extrinsics.r33));
    transforms_->sendTransform(cloud_transform);
    pcl::toROSMsg(*current_cloud, cloud_msg_);
    cloud_msg_.header.frame_id = base_link_ + "/cloud_frame";
    cloud_msg_.header.stamp = ros::Time(timeStampToRosTime(data.timeOfCapture));
    pub_cloud_.publish (cloud_msg_);
    //g_context.quit();
  }
  
  void SoftKineticsCamera::configureAudioNode(){
    g_anode_.newSampleReceivedEvent().connect(this,&SoftKineticsCamera::onNewAudioSample);
    AudioNode::Configuration config = g_anode_.getConfiguration();
    config.sampleRate = 44100;
    try{
        g_context_.requestControl(g_anode_,0);
        g_anode_.setConfiguration(config);
        g_anode_.setInputMixerLevel(0.5f);
    }
    catch (ArgumentException& e){
        ROS_ERROR("Argument Exception: %s",e.what());
    }
    catch (UnauthorizedAccessException& e){
        ROS_ERROR("Unauthorized Access Exception: %s",e.what());
    }
    catch (ConfigurationException& e){
        ROS_ERROR("Configuration Exception: %s",e.what());
    }
    catch (StreamingException& e){
        ROS_ERROR("Streaming Exception: %s",e.what());
    }
    catch (TimeoutException&){
        ROS_ERROR("TimeoutException");
    }
  }


  void SoftKineticsCamera::configureDepthNode(){
    g_dnode_.newSampleReceivedEvent().connect(this,&SoftKineticsCamera::onNewDepthSample);
    DepthNode::Configuration config = g_dnode_.getConfiguration();

    config.frameFormat = depth_frame_format_;
    config.framerate = depth_frame_rate_;
    config.mode = depth_mode_;
    config.saturation = false;
    try{
        g_context_.requestControl(g_dnode_,0);
        g_dnode_.setEnableVertices(true);
        g_dnode_.setEnableConfidenceMap(true);
        g_dnode_.setConfidenceThreshold(confidence_threshold_);
        g_dnode_.setEnableVerticesFloatingPoint(true);
        g_dnode_.setEnableDepthMapFloatingPoint(true);
        g_dnode_.setConfiguration(config);
    }
    catch (ArgumentException& e){
        ROS_ERROR("Argument Exception: %s",e.what());
    }
    catch (UnauthorizedAccessException& e){
        ROS_ERROR("Unauthorized Access Exception: %s",e.what());
    }
    catch (IOException& e){
        ROS_ERROR("IO Exception: %s",e.what());
    }
    catch (InvalidOperationException& e){
        ROS_ERROR("Invalid Operation Exception: %s",e.what());
    }
    catch (ConfigurationException& e){
        ROS_ERROR("Configuration Exception: %s",e.what());
    }
    catch (StreamingException& e){
        ROS_ERROR("Streaming Exception: %s",e.what());
    }
    catch (TimeoutException&){
        ROS_ERROR("TimeoutException");
    }
  }
  
  void SoftKineticsCamera::configureColorNode(){
    // connect new color sample handler
    g_cnode_.newSampleReceivedEvent().connect(this, &SoftKineticsCamera::onNewColorSample);

    ColorNode::Configuration config = g_cnode_.getConfiguration();

    config.frameFormat = color_frame_format_;
    config.compression = color_compression_;
    config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
    config.framerate = color_frame_rate_;
    g_cnode_.setEnableColorMap(true);

    try{
        g_context_.requestControl(g_cnode_,0);
        g_cnode_.setConfiguration(config);
    }
    catch (ArgumentException& e){
        ROS_ERROR("Argument Exception: %s",e.what());
    }
    catch (UnauthorizedAccessException& e){
        ROS_ERROR("Unauthorized Access Exception: %s",e.what());
    }
    catch (IOException& e){
        ROS_ERROR("IO Exception: %s",e.what());
    }
    catch (InvalidOperationException& e){
        ROS_ERROR("Invalid Operation Exception: %s",e.what());
    }
    catch (ConfigurationException& e){
        ROS_ERROR("Configuration Exception: %s",e.what());
    }
    catch (StreamingException& e){
        ROS_ERROR("Streaming Exception: %s",e.what());
    }
    catch (TimeoutException&){
        ROS_ERROR("TimeoutException");
    }
  }

  void SoftKineticsCamera::configureNode(Node node){
    if ((node.is<DepthNode>())&&(!g_dnode_.isSet())&&(depth_enabled_)){
        g_dnode_ = node.as<DepthNode>();
        configureDepthNode();
        g_context_.registerNode(node);
    }

    if ((node.is<ColorNode>())&&(!g_cnode_.isSet())&&(color_enabled_)){
        g_cnode_ = node.as<ColorNode>();
        configureColorNode();
        g_context_.registerNode(node);
    }

    if ((node.is<AudioNode>())&&(!g_anode_.isSet())&&(audio_enabled_)){
        g_anode_ = node.as<AudioNode>();
        configureAudioNode();
        g_context_.registerNode(node);
    }
  }  

  void SoftKineticsCamera::onNodeConnected(Device device, Device::NodeAddedData data){
    configureNode(data.node);
  }

  void SoftKineticsCamera::onNodeDisconnected(Device device, 
    Device::NodeRemovedData data){
    if (data.node.is<AudioNode>() && (data.node.as<AudioNode>() == g_anode_))
        g_anode_.unset();
    if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode_))
        g_cnode_.unset();
    if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode_))
        g_dnode_.unset();
    ROS_INFO("Depth && Color && Audio nodes disconnected");
  }

  void SoftKineticsCamera::onDeviceConnected(Context context, 
    Context::DeviceAddedData data){
    if (!g_bDeviceFound_){
        data.device.nodeAddedEvent().connect(this,&SoftKineticsCamera::onNodeConnected);
        data.device.nodeRemovedEvent().connect(this,&SoftKineticsCamera::onNodeDisconnected);
        g_bDeviceFound_ = true;
    }
  }

  void SoftKineticsCamera::onDeviceDisconnected(Context context, 
    Context::DeviceRemovedData data){
    g_bDeviceFound_ = false;
    ROS_INFO("Device disconnected");
  }
