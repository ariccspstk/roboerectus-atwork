#ifndef CONNECTION_BASED_NODELET_H_
#define CONNECTION_BASED_NODELET_H_
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>

namespace aricc_topic_tools
{
	/** @brief
	 * Nodelet to automatically subscribe/unsubscribe
	 * topics according to subscription of advertised topics.
	 *
	 * It's important not to subscribe topic if no output is required.
	 *
	 * In order to watch advertised topics, need to use advertise template method.
	 * And create subscribers in subscribe() and shutdown them in unsubscribed().
	 *
	 */
  class ConnectionBasedNodelet: public nodelet::Nodelet{
  public:
    ConnectionBasedNodelet(): subscribed_(false) { }
  protected:
  /** @brief
   * Initialize private nodehandle pnh_. Subclass should call
   * this method in its onInit method
   */
    virtual void onInit();
  /** @brief
   * callback function which is called when new subscriber come
   */
    virtual void connectionCallback(const ros::SingleSubscriberPublisher& pub);
  /** @brief
   * This method is called when publisher is subscribed by other
   * nodes.
   * Set up subscribers in this method.
   */
    virtual void subscribe() = 0;
  /** @brief
   * This method is called when publisher is unsubscribed by other
   * nodes.
   * Shut down subscribers in this method.
   */
    virtual void unsubscribe() = 0;
  /** @brief
   * Advertise a topic and watch the publisher. Publishers which are
   * created by this method.
   *
   * @param nh NodeHandle.
   * @param topic topic name to advertise.
   * @param queue_size queue size for publisher.
   * @return Publisher for the advertised topic.
   */
    template<class T> 
    ros::Publisher advertise(ros::NodeHandle& nh,
                             std::string topic, int queue_size){ 
      ros::SubscriberStatusCallback connect_cb = boost::bind( &ConnectionBasedNodelet::connectionCallback, this, _1);
      ros::SubscriberStatusCallback disconnect_cb = boost::bind( &ConnectionBasedNodelet::connectionCallback, this, _1);
      ros::Publisher ret = nh.advertise<T>(topic, queue_size,
					   connect_cb,
					   disconnect_cb);
      publishers_.push_back(ret);
      return ret;
    }
  /** @brief
   * mutex to call subscribe() and unsubscribe() in
   * critical section.
   */
    boost::mutex connection_mutex_;
  /** @brief
   * List of watching publishers
  */
    std::vector<ros::Publisher> publishers_;
  /** @brief
   * Shared pointer to private nodehandle.
   */
    boost::shared_ptr<ros::NodeHandle> pnh_;
  /** @brief
   * A flag to check if any publisher is already subscribed
   * or not.
   */
    bool subscribed_;
  private:
  };
}
#endif
