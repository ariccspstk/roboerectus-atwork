#ifndef ARICC_3D_VISION_PASS_THROUGH_FILTER_H_
#define ARICC_3D_VISION_PASS_THROUGH_FILTER_H_
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <aricc_topic_tools/connection_based_nodelet.h>
#include "aricc_3d_vision/pcl_conversion_util.h"
#include <aricc_3d_vision/PassThroughFilterConfig.h>
#include <dynamic_reconfigure/server.h>

namespace aricc_3d_vision
{
  class PassThroughFilter: public aricc_topic_tools::ConnectionBasedNodelet{
	public:
		typedef PassThroughFilterConfig Config;

	protected:
		virtual void onInit();
		virtual void process(const sensor_msgs::PointCloud2::ConstPtr& msg);
		virtual void subscribe();
		virtual void unsubscribe();
		virtual void configCallback(Config &config, uint32_t level);

		ros::Publisher pub_;
		ros::Subscriber sub_;
                boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
		boost::mutex mutex_;
                std::string field_name_;
		double max_;
		double min_;
	private:
	};
}
#endif
