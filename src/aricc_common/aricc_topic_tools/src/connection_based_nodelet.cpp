#include "aricc_topic_tools/connection_based_nodelet.h"

namespace aricc_topic_tools
{
  void ConnectionBasedNodelet::onInit(){
    pnh_.reset (new ros::NodeHandle (getMTPrivateNodeHandle ()));
  }

  void ConnectionBasedNodelet::connectionCallback(const ros::SingleSubscriberPublisher& pub){
    boost::mutex::scoped_lock lock(connection_mutex_);
    for (size_t i = 0; i < publishers_.size(); i++){
      ros::Publisher pub = publishers_[i];
      if (pub.getNumSubscribers() > 0) {
        if (!subscribed_) {
	  subscribe();
	  subscribed_ = true;
        }
        return;
      }
    }
    if (subscribed_) {
      unsubscribe();
      subscribed_ = false;
    }
  }
}
