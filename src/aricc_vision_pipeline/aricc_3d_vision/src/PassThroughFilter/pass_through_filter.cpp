#include "aricc_3d_vision/pass_through_filter.h"
#include <pluginlib/class_list_macros.h>

namespace aricc_3d_vision
{
	void PassThroughFilter::onInit()
	{
		ConnectionBasedNodelet::onInit();
		pnh_->param("field_name", field_name_, std::string("x"));
		pnh_->param("min", min_, 0.0);
		pnh_->param("max", max_, 1.0);
                //Reconfigure
                srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
                typename dynamic_reconfigure::Server<Config>::CallbackType f = boost::bind (&PassThroughFilter::configCallback, this, _1, _2);
                srv_->setCallback (f);
		pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
	}
        
	void PassThroughFilter::process(const sensor_msgs::PointCloud2::ConstPtr& msg){
          boost::mutex::scoped_lock lock(mutex_);
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
          std_msgs::Header header = msg->header;
          pcl::fromROSMsg(*msg, *pcl_cloud);
          // Create the filtering object
          pcl::PassThrough<pcl::PointXYZRGB> pass;
          pass.setKeepOrganized(true);
          pass.setInputCloud (pcl_cloud);
          pass.setFilterFieldName (field_name_);
          pass.setFilterLimits (min_, max_);
          //pass.setFilterLimitsNegative (true);
          pass.filter (*pcl_cloud);
          
          sensor_msgs::PointCloud2 ros_cloud;
          pcl::toROSMsg(*pcl_cloud, ros_cloud);
          ros_cloud.header = header;
	  pub_.publish(ros_cloud);
	}
	void PassThroughFilter::subscribe(){
	  sub_ = pnh_->subscribe("input",1,&PassThroughFilter::process, this);
	}

	void PassThroughFilter::unsubscribe(){
	  sub_.shutdown();
	}

        void PassThroughFilter::configCallback(Config &config, uint32_t level){
	  boost::mutex::scoped_lock lock(mutex_);
          field_name_ = config.field_name;
          min_ = config.min;
          max_ = config.max;
	}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aricc_3d_vision::PassThroughFilter, nodelet::Nodelet);
