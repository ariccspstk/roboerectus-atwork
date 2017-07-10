#include <stdlib.h>
#include <ros/ros.h>
#include <softkinetics_camera_ros/softkinetics_camera_driver.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "softkinetics_camera_node");
    ROS_INFO_STREAM("----SoftKinetic DepthSense Node----\r");
    SoftKineticsCamera camera;
    camera.start();
    ROS_INFO( "... shutting down complete." );
    return 0;
}
