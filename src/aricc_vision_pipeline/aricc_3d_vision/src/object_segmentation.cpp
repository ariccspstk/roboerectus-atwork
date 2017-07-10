/*
 *****************************************************************
 *   ROS package name: aricc_3d_vision
 *   Author: Ian Wang
 *   Date of creation: March 2015
 *
 *****************************************************************
*/

//C/C++ header
#include <stdio.h>
#include <vector>
#include <exception>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <string>

//ROS header
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

//PCL header
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/transforms.h>

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <cv_bridge/cv_bridge.h>

//Dynamic reconfiguration
#include <aricc_3d_vision/SegmentationConfig.h>
#include <dynamic_reconfigure/server.h>

struct clusterCloud{
  pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
  double cp_x;
  double cp_y;
  bool visited;
};

using namespace std;

ros::Subscriber _cloudSub;
ros::Publisher _cloudPub;
ros::Publisher _clusterPub;
ros::Publisher _tablePub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloudInPtr;
vector< pcl::PointCloud<pcl::PointXYZRGB> > _clusterExtracted;

//Parameters

//Pass Through Filter
bool _use_pass_through_filter;
string _ps_field_name;
double _ps_limit_min, _ps_limit_max;

//Cloud Radius Filter
bool _use_cloud_radius_filter;
double _cr_search_radius;
int _cr_min_radius;

//Down sample voxel
double _voxel_grid_side;
bool _use_voxel_grid_filter;

//Planar Segmentation
int _ps_max_iterations;
int _inlier_threshold;
double _ps_dist_threshold;

//Normal Segmentation
double _ns_dist_weight;
double _ns_dist_threshold;

//Polygonal PrismDat
double _pp_height_limit_min;
double _pp_height_limit_max;

//Euclidean Cluster Extraction
double _ece_tolerance;
int _ece_cluster_min;
int _ece_cluster_max;

bool _received_point_cloud;
string _camera_link;
string _table_link;

void configureCB(aricc_3d_vision::SegmentationConfig &config, 
                 uint32_t level){
  //_use_pass_through_filter = config.use_pass_through_filter;
  //_ps_field_name = config.ps_field_name;
  //_ps_limit_min = config.ps_limit_min;
  //_ps_limit_max = config.ps_limit_max;

  _use_cloud_radius_filter = config.use_cloud_radius_filter;
  _cr_search_radius = config.cr_search_radius;
  _cr_min_radius = config.cr_min_radius;

  _ns_dist_weight = config.ns_dist_weight;
  _ns_dist_threshold = config.ns_dist_threshold; 

  _voxel_grid_side = config.voxel_grid_side;
  _use_voxel_grid_filter = config.use_voxel_grid_filter;

  _ps_max_iterations = config.ps_max_iterations;
  _ps_dist_threshold = config.ps_dist_threshold;

  _pp_height_limit_min = config.pp_height_limit_min;
  _pp_height_limit_max = config.pp_height_limit_max;

  _ece_tolerance = config.ece_tolerance;
  _ece_cluster_min = config.ece_cluster_min;
  _ece_cluster_max = config.ece_cluster_max;
}

void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& msgPtr){
  pcl::fromROSMsg(*msgPtr,*_cloudInPtr);
  if(!_received_point_cloud) _received_point_cloud = true;
}

void publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn,
                  ros::Publisher cloudPub)
{
  //convert current_cloud to PointCloud2 and publish
  sensor_msgs::PointCloud2 cloudROSMsg;
  pcl::toROSMsg(*cloudIn, cloudROSMsg);
  cloudROSMsg.header.stamp = ros::Time::now();
  cloudROSMsg.header.frame_id = cloudIn->header.frame_id; 
  cloudPub.publish(cloudROSMsg);
}

void psFilterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn){
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloudIn);
  pass.setFilterLimitsNegative (false);
  
  //FieldX
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.20, 0.20);
  pass.filter(*cloudIn);
  //FieldY
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.10, 0.10);
  pass.filter(*cloudIn);
  //FieldZ
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (_ps_limit_min, _ps_limit_max);
  pass.filter(*cloudIn);
}

void crFilterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn){
    // radius based filter:
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
    ror.setInputCloud(cloudIn);
    ror.setRadiusSearch(_cr_search_radius);
    ror.setMinNeighborsInRadius(_cr_min_radius);
    // apply filter
    ror.filter(*cloudIn);
}

void downsampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn){
  //ROS_DEBUG_STREAM("Starting downsampling");
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloudIn);
  sor.setLeafSize(_voxel_grid_side, _voxel_grid_side, _voxel_grid_side);
  sor.setFilterFieldName("z");
  sor.setFilterLimits(0, 0.4);
  sor.filter(*cloudIn);
  //ROS_DEBUG_STREAM("downsampled!");
}

void planeTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInPtr,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOutPtr,
                    pcl::ModelCoefficients::Ptr coePtr){
  ROS_ASSERT(coePtr->values.size() > 3);
  double a = coePtr->values[0];
  double b = coePtr->values[1];
  double c = coePtr->values[2];
  double d = coePtr->values[3];

  //position
  double posX = -a*d;
  double posY = -b*d;
  double posZ = -c*d;
  
  //asume plane coefficients are normalized
  tf::Vector3 position(-a*d, -b*d, -c*d);
  tf::Vector3 z(a, b, c);
 
  // rotation around z axis
  double rotZ = -atan2( a, b);
  // rotation around y axis
  double rotY;
  if(rotZ  == 0) rotY = -atan2(a,c);
  else
    rotY = -atan2( c,a);
  // rotation around x axis
  double rotX = atan2( c,b);
  rotX  += M_PI_2;
  double wat = atan2(0,0);

  //std::cout<< "positions are: "<< posX <<","<< posY <<"," <<posZ <<std::endl;
  //std::cout<< "angles are: "<< rotX <<","<< rotY <<"," <<rotZ <<","<<wat<< std::endl;
  Eigen::Matrix3f rot_Matrix_z; // transition matrix_z
  Eigen::Matrix3f rot_Matrix_y; // transition matrix_y
  Eigen::Matrix3f rot_Matrix_x; // transition matrix_x
  Eigen::Matrix3f Rot_Matrix; // transition matrix_x

  //rotX = 0.0;  
  rot_Matrix_x << 1, 0, 0,
                  0, cos(rotX), -sin(rotX),
                  0, sin(rotX), cos(rotX);
  
  rot_Matrix_y << cos(rotY), -sin(rotY), 0,
                  0, 1, 0,
                  sin(rotY), cos(rotY), 0;
  
  rot_Matrix_z << cos(rotZ), -sin(rotZ), 0,
                  sin(rotZ), cos(rotZ), 0,
                  0, 0, 1;
  Rot_Matrix = rot_Matrix_x;
  //Rot_Matrix = rot_Matrix_x*rot_Matrix_y;
  
  tf::Matrix3x3 rotation;
  for(unsigned int i=0; i<3; i++)
    for(unsigned int j=0; j<3; j++)
      rotation[i][j] =  Rot_Matrix(i,j);

  tf::Quaternion orientation;
  rotation.getRotation(orientation);
  tf::Transform planeTransform = tf::Transform(orientation, position);

  //------ Broadcasting table plane link ------//
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(planeTransform, ros::Time::now(), _camera_link, _table_link));

  //------ Transform points to table link ------//
  pcl_ros::transformPointCloud(*cloudInPtr, *cloudOutPtr, planeTransform.inverse());
  cloudOutPtr->header.frame_id = _table_link;
}

void rotationCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInPtr, 
                   pcl::ModelCoefficients::Ptr coePtr){
// build the Euler angle roation matrix from the plane coefficent aX + bY + cZ + D = 0
// Rz(a) = [cos(a) -sin(a) 0; Ry(a) = [cos(a) 0 sin(a); Rx(a) = [1 0 0;
// sin(a) cos(a) 0; 0 1 0 ; 0 cos(a) -sin(a);
// 0 0 1] -sin(a) 0 cos(a)] 0 sin(a) cos(a)]
// General rotation matrix
// Rz(a1)R(a2)R(a3) = [cos(a2)cos(a1) -cos(a3)sin(a1)+sin(a3)sin(a2)cos(a1) sin(a3)sin(a1)+cos(a3)sin(a2)cos(a1);
// cos(a2)sin(a1) cos(a3)cos(a1)+sin(a3)sin(a2)sin(a1) -sin(a3)cos(a1)+cos(a3)sin(a2)sin(a1);
// -sin(a2) sin(a3)sin(a2) cos(a3)cos(a2) ]
// compute angle
  
  ROS_INFO("Starting rotating points");
  Eigen::VectorXf coefficient(4);
  coefficient[0] = coePtr->values[0];
  coefficient[1] = coePtr->values[1];
  coefficient[2] = coePtr->values[2];
  coefficient[3] = coePtr->values[3];
  std::cout<< "coefficient are: "<< coefficient[0] <<","<< coefficient[1] <<"," <<coefficient[2] <<","<<coefficient[3]<< std::endl;

  // rotation around z axis
  float rotZ = -atan2( coefficient[0],coefficient[1]);
  // rotation around y axis
  float rotY;
  if(rotZ  == 0) rotY = -atan2(coefficient[0],coefficient[2]);
  else
    rotY = -atan2( coefficient[2],coefficient[0]); 
  // rotation around x axis
  float rotX = -atan2( coefficient[2],coefficient[1]);
  float wat = atan2(0,0);
  std::cout<< "angles are: "<< rotZ <<","<< rotY <<"," <<rotX <<","<<wat<< std::endl;
  
  Eigen::Matrix3f rot_Matrix_z; // transition matrix_z
  Eigen::Matrix3f rot_Matrix_y; // transition matrix_y
  Eigen::Matrix3f rot_Matrix_x; // transition matrix_x
  Eigen::Matrix3f Rot_Matrix; // transition matrix_x
  rot_Matrix_z << cos(rotZ), -sin(rotZ), 0,
                  sin(rotZ), cos(rotZ), 0,
                  0, 0, 1;

  rot_Matrix_x << 1, 0, 0,
                  0, cos(rotX), -sin(rotX),
                  0, sin(rotX), cos(rotX);
  Rot_Matrix = rot_Matrix_z * rot_Matrix_x;
  // move all the points along the direction of the normal of the plane with "d_t_0", where d is the distance from original point (0,0,0) to the plane
  float d_t_0 = coefficient[3];
  Eigen::Vector3f point;
  Eigen::Vector3f trans_d;
  //normalize the plane norm
  Eigen::Vector3f trans_dd;
  Eigen::Vector4f coefficient_norm;
  coefficient_norm = coefficient;
  float sq_sum_1 = coefficient_norm.norm();
  std::cout<< "norm is "<< sq_sum_1<<std::endl;
  //--------------------------------------------------------
  Eigen::Vector3f coefficient_norm_1;
  //float sq_sum = sqrt(coefficient[0]*coefficient[0]+coefficient[1]*coefficient[1]+coefficient[2]*coefficient[2]+coefficient[3]*coefficient[3]);
  float sq_sum = sqrt(coefficient[0]*coefficient[0]+coefficient[1]*coefficient[1]+coefficient[2]*coefficient[2]);
  coefficient_norm_1[0] = coefficient[0]/sq_sum;
  coefficient_norm_1[1] = coefficient[1]/sq_sum;
  coefficient_norm_1[2] = coefficient[2]/sq_sum;
  d_t_0 = coefficient[3];
  trans_d = coefficient_norm_1 * d_t_0;
  
  // point container
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOutPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB cloudTemp;
  for(unsigned int i = 0; i< cloudInPtr->points.size();i ++){
    point << cloudInPtr->points[i].x,
             cloudInPtr->points[i].y,
             cloudInPtr->points[i].z;
    point = Rot_Matrix * point;
    point = point + trans_d;
    cloudTemp.x = point[0];
    cloudTemp.y = point[1];
    cloudTemp.z = point[2];
    cloudTemp.r = cloudInPtr->points[i].r;
    cloudTemp.g = cloudInPtr->points[i].g;
    cloudTemp.b = cloudInPtr->points[i].b;
    cloudOutPtr->points.push_back(cloudTemp);
  }
  *cloudInPtr = *cloudOutPtr;
}

int planeModelNormalSegment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn){
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloudNormalsPtr (new pcl::PointCloud<pcl::Normal>);
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB> ());
  
  ne.setInputCloud (cloudIn);
  ne.setSearchMethod (tree);
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.01);
  // Compute the features
  ne.compute (*cloudNormalsPtr);
  
  // Table model fitting parameters
  seg.setDistanceThreshold (_ns_dist_threshold);
  seg.setMaxIterations (1000);
  seg.setNormalDistanceWeight (_ns_dist_weight);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setProbability (0.99); 
  seg.setInputCloud(cloudIn);
  seg.setInputNormals (cloudNormalsPtr);
  seg.segment(*inliers, *coefficients);

  if(coefficients->values.size () <=3){
    ROS_ERROR("Failed to detect table in scan");
    return 0;
  }
  if( inliers->indices.size() < (unsigned int)_inlier_threshold){
    ROS_ERROR("Plane detection has %d inliers, below min threshold of %d", (int)inliers->indices.size(),_inlier_threshold);
    return 0;
  }

  //extract objects
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractObjectsPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
  extract.setInputCloud (cloudIn);
  extract.setNegative (true);
  extract.setIndices(inliers);
  extract.filter(*extractObjectsPtr);
  *cloudIn = *extractObjectsPtr;
  publishCloud(cloudIn,_tablePub);

  /* 
  //Project the table inliers on the table
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tableProjectedPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloudIn);
  proj.setIndices(inliers);
  proj.setModelCoefficients(coefficients);
  proj.filter (*tableProjectedPtr);
  
  //Estimate the convex hull for table
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tableConvexHullPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConvexHull<pcl::PointXYZRGB> hull;
  hull.setInputCloud(tableProjectedPtr);
  // Make sure that the resulting hull is bidimensional.
  //hull.setDimension(2);
  hull.reconstruct(*tableConvexHullPtr);
  // *cloudIn = *tableConvexHullPtr;
  //Rotate points to table plane
  //rotationCloud(cloudIn,coefficients);
  
  if(hull.getDimension() != 2){
    ROS_ERROR("The input cloud does not represent a planar surface");
    return 0;
  }
  
  // Transform cloud to plane coordinate
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPlaneTransPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
  //planeTransform(cloudIn, cloudPlaneTransPtr, coefficients);
  //planeTransform(tableConvexHullPtr, tableConvexHullPtr, coefficients);

  //publishCloud(cloudPlaneTransPtr,_tablePub); 
  //publishCloud(tableConvexHullPtr,_tablePub); 
  //publishCloud(tableProjectedPtr,_tablePub); 

  // ------ Prism object ------//
  //In ExtractPolygonalPrismData, the points have been transformed
  //based on hull, so no need to transform points again.
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
  prism.setInputCloud(cloudIn);
  prism.setInputPlanarHull(tableConvexHullPtr);
  ROS_INFO("%lf, %lf",_pp_height_limit_min,_pp_height_limit_max);
  prism.setHeightLimits(_pp_height_limit_min, _pp_height_limit_max);
  pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);
  prism.segment(*objectIndices);
  ROS_INFO("FoundObject:%lu",objectIndices->indices.size());
  
  //extract objects
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractObjectsPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
  extract.setInputCloud (cloudIn);
  extract.setNegative (false);
  extract.setIndices(objectIndices);
  extract.filter(*extractObjectsPtr);
  *cloudIn = *extractObjectsPtr;
  publishCloud(cloudIn,_tablePub);
  */ 
}

void planeModelSegment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn){
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB> ());
  //pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (_ps_max_iterations);
  seg.setDistanceThreshold (_ps_dist_threshold);

  unsigned int nr_points = (int)cloudIn->points.size();
  ROS_INFO("Input Cloud point size:%d",nr_points);
  while (cloudIn->points.size () > 0.3 * nr_points){
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloudIn);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size () == 0){
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloudIn);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloudIn = *cloud_f;
  }
  cout<<""<<endl;
  //publishCloud(cloudIn,"After_extract");
}

void euclideanSegment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn,
                      vector< pcl::PointCloud<pcl::PointXYZRGB> >& res){
  ROS_INFO("PointCloudIn:%lu",cloudIn->points.size());
  std::vector<clusterCloud> clustersOut;
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloudIn);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (_ece_tolerance); // 2cm
  ec.setMinClusterSize (_ece_cluster_min);
  ec.setMaxClusterSize (_ece_cluster_max);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloudIn);
  ec.extract(cluster_indices);

  //unsigned int i = 0;
  res.clear();
  clustersOut.clear();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZRGB>);
  ROS_INFO("Found Cluster Number:%lu",cluster_indices.size());
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    //ROS_INFO("Cluster size:%lu",it->indices.size());
    //if(it->indices.size() > 700) continue;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    clusterCloud oneCluster;
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end (); ++pit){
      //ROS_INFO("%d",*pit);
      pcl::PointXYZRGB tempPoint = cloudIn->points[*pit];
      oneCluster.pointCloud.points.push_back(tempPoint);
      oneCluster.cp_x += tempPoint.x;
      oneCluster.cp_y += tempPoint.y;

      //cloud_cluster->points.push_back (tempPoint); 
      //cloud_clusters->points.push_back (tempPoint);
    }
    oneCluster.pointCloud.width = oneCluster.pointCloud.points.size();
    oneCluster.pointCloud.height = 1;
    oneCluster.pointCloud.is_dense = true;
    oneCluster.pointCloud.header.frame_id = _camera_link;
    oneCluster.cp_x /= oneCluster.pointCloud.points.size();
    oneCluster.cp_y /= oneCluster.pointCloud.points.size();
    oneCluster.visited = false;
    clustersOut.push_back(oneCluster);    

    //ROS_INFO("Cluster size:%lu",cloud_cluster->points.size());
    //cloud_cluster->width = cloud_cluster->points.size();
    //cloud_cluster->height = 1;
    //cloud_cluster->is_dense = true;
    //cloud_cluster->header.frame_id = _camera_link; 
    //clusterOut.push_back(*cloud_cluster);
    //publishCloud(cloud_cluster,_clusterPub);
  }
  
  //Publish cluster for visulization purpose
  //ROS_INFO("Cluster size:%lu",cloud_clusters->points.size());
  //cloud_clusters->width = cloud_clusters->points.size ();
  //cloud_clusters->height = 1;
  //cloud_clusters->is_dense = true;
  //cloud_clusters->header.frame_id = _camera_link;
  //publishCloud(cloud_clusters,_clusterPub);

  //Merge Closer Cluster together
  for(unsigned int i = 0; i < clustersOut.size(); ++i){
    if(clustersOut.at(i).visited) continue;
    pcl::PointCloud<pcl::PointXYZRGB> tempCloud;
    tempCloud = clustersOut.at(i).pointCloud;
    for(unsigned int j = (i+1); j < clustersOut.size(); ++j){
      if(clustersOut.at(j).visited) continue;
      double dx = clustersOut.at(i).cp_x - clustersOut.at(j).cp_x;
      double dy = clustersOut.at(i).cp_y - clustersOut.at(j).cp_y;
      double dist = sqrt( dx*dx + dy*dy );
      ROS_INFO("Dist:%lf",dist);
      if(dist < 0.07) {
        tempCloud += clustersOut.at(j).pointCloud;
        clustersOut.at(j).visited = true;
      }
    }
    ROS_INFO("");
    res.push_back(tempCloud);
  }

  ROS_INFO("Cluster Number:%lu",res.size());
  //Color point cloud for deguging
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTempPtr(new pcl::PointCloud<pcl::PointXYZRGB>); 
  for(unsigned int i = 0; i < res.size(); ++i){
    //Randomly generate color
    uint8_t r = random()%255;
    uint8_t g = random()%255;
    uint8_t b = random()%255;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    for(unsigned int j = 0; j < res.at(i).points.size(); ++j){
      res.at(i).points.at(j).rgb = *reinterpret_cast<float*>(&rgb);
      cloudTempPtr->push_back(res.at(i).points.at(j));
    }
  }
  cloudTempPtr->width = cloudTempPtr->points.size ();
  cloudTempPtr->height = 1;
  cloudTempPtr->is_dense = true;
  cloudTempPtr->header.frame_id = _camera_link;
  publishCloud(cloudTempPtr,_clusterPub);
}

void process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn){
  if(!_received_point_cloud) return;
  if(_received_point_cloud) _received_point_cloud = false;
  if(_use_pass_through_filter) psFilterCloud(cloudIn);
  if(_use_cloud_radius_filter) crFilterCloud(cloudIn);
  if(_use_voxel_grid_filter) downsampleCloud(cloudIn);
  planeModelNormalSegment(cloudIn);
  //planeModelSegment(cloudIn);
  //publishCloud(cloudIn,_cloudPub);
  euclideanSegment(cloudIn,_clusterExtracted); 
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "aricc_3d_vision");
  ros::NodeHandle nh("~");
  
  //initialize dynamic reconfigure server
  dynamic_reconfigure::Server<aricc_3d_vision::SegmentationConfig> dr_server;
  dynamic_reconfigure::Server<aricc_3d_vision::SegmentationConfig>::CallbackType f;
  f = boost::bind(&configureCB, _1, _2);
  dr_server.setCallback(f);

  //initialize parameters
  string cloud_sub_topic, cloud_pub_topic;
  nh.param<bool>("use_pass_through_filter", _use_pass_through_filter, true);
  nh.param<string>("pass_through_field_name", _ps_field_name, "z");
  nh.param<double>("pass_through_limit_min", _ps_limit_min, 0.2);
  nh.param<double>("pass_through_limit_max", _ps_limit_max, 0.4);
  
  nh.param<double>("height_limit_min", _pp_height_limit_min, 0.0);
  nh.param<double>("height_limit_max", _pp_height_limit_max, 0.1);
  
  nh.param<double>("ns_dist_weight", _ns_dist_weight, 0.1);
  nh.param<double>("ns_dist_threshold", _ns_dist_threshold, 0.02);

  nh.param<bool>("use_cloud_radius_filter", _use_cloud_radius_filter, false);
  nh.param<double>("cr_search_radius", _cr_search_radius, 0.01);
  nh.param<int>("cr_min_radius", _cr_min_radius, 2);
  
  nh.param<bool>("use_voxel_grid_filter",_use_voxel_grid_filter, false);
  nh.param<double>("voxel_grid_side", _voxel_grid_side, 0.01);

  
  nh.param<int>("planar_segment_max_it", _ps_max_iterations, 100);
  nh.param<int>("inlier_threshold", _inlier_threshold, 100);
  nh.param<double>("planar_segment_dist_threshold", _ps_dist_threshold, 0.01);


  nh.param<double>("euclidean_cluster_tolerance", _ece_tolerance, 0.01);
  nh.param<int>("euclidean_cluster_min", _ece_cluster_min, 100);
  nh.param<int>("euclidean_cluster_max", _ece_cluster_max, 25000);

  nh.param<std::string>("cloud_sub_topic",cloud_sub_topic,"point_cloud");
  nh.param<std::string>("camera_link", _camera_link, "base_rgbd_camera_link");
  nh.param<std::string>("table_link", _table_link, "base_table_link");
  
  _received_point_cloud = false;
  _cloudInPtr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  //initialize publishers and subscribers
  _cloudPub = nh.advertise<sensor_msgs::PointCloud2>("Cloud_Filted", 1000);
  _tablePub = nh.advertise<sensor_msgs::PointCloud2>("Cloud_table", 1000);
  _clusterPub = nh.advertise<sensor_msgs::PointCloud2>("Cloud_Cluster", 1000);
  _cloudSub = nh.subscribe(cloud_sub_topic,1000,cloudCB);


  //Loop rate in Hz
  ros::Rate rate(10);
  while(nh.ok()){
    ros::spinOnce();
    process(_cloudInPtr);
    rate.sleep();
  } 
  //Shutdown ros processing
  ros::shutdown();
  return 0;
}



