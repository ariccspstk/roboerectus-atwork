#ifndef ARICC_LASER_PIPELINE_GET_BEAM_DIST_H_
#define ARICC_LASER_PIPELINE_GET_BEAM_DIST_H_

#include <aricc_topic_tools/connection_based_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <aricc_laser_pipeline/BeamDistArray.h>
#include <aricc_utils/geometry_utils.h>

namespace aricc_laser_pipeline{
  class GetBeamDist: public aricc_topic_tools::ConnectionBasedNodelet{
  public:
  
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
  
  private:
    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::vector<double> angle_list_;

    void execute( const sensor_msgs::LaserScan::ConstPtr& msg);
    bool loadAngleList( std::string name, std::vector<double>& angle_list);
    bool getDist( sensor_msgs::LaserScan reading, double angle, double& dist, unsigned int beam);
  };
}

#endif
