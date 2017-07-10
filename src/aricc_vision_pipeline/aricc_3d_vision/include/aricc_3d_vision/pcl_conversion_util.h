#ifndef ARICC_3D_VISION_PCL_CONVERSION_UTIL_H_
#define ARICC_3D_VISION_PCL_CONVERSION_UTIL_H_
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Point32.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/range_image/range_image_planar.h>
#if ROS_VERSION_MINIMUM(1, 10, 0)
// hydro and later
typedef pcl_msgs::PointIndices PCLIndicesMsg;
typedef pcl_msgs::ModelCoefficients PCLModelCoefficientMsg;
#else
// groovy
typedef pcl::PointIndices PCLIndicesMsg;
typedef pcl::ModelCoefficients PCLModelCoefficientMsg;
#endif
#include <opencv2/opencv.hpp>
namespace aricc_3d_vision
{
	/** @brief
	 * Convert pcl::RangeImage to cv::Mat. Distance is normalized
	 * to 0-1 and colorized.
	 *
	 * @param range_image instance of pcl::RangeImage
	 * @param mat instance of cv::Mat, converted cv::Mat is set into
	 * this argument.
	 */
	void rangeImageToCvMat(const pcl::RangeImage& range_image,
			cv::Mat& mat);
	template<class FromT, class ToT>
		void pointFromXYZToVector(const FromT& msg,
				ToT& p)
		{
			p[0] = msg.x; p[1] = msg.y; p[2] = msg.z;
		}
	template<class FromT, class ToT>
		void pointFromVectorToXYZ(const FromT& p,
				ToT& msg)
		{
			msg.x = p[0]; msg.y = p[1]; msg.z = p[2];
		}
	template<class FromT, class ToT>
		void pointFromXYZToXYZ(const FromT& from,
				ToT& to)
		{
			to.x = from.x; to.y = from.y; to.z = from.z;
		}
	template<class FromT, class ToT>
		void pointFromVectorToVector(const FromT& from,
				ToT& to)
		{
			to[0] = from[0]; to[1] = from[1]; to[2] = from[2];
		}
	template<class FromT, class ToT>
		void convertMatrix4(const FromT& from,
				ToT& to)
		{
			for (size_t i = 0; i < 4; i++) {
				for (size_t j = 0; j < 4; j++) {
					to(i, j) = from(i, j);
				}
			}
		}
	void convertEigenAffine3(const Eigen::Affine3d& from,
			Eigen::Affine3f& to);
	void convertEigenAffine3(const Eigen::Affine3f& from,
			Eigen::Affine3d& to);
}
// extend pcl_conversions package's toPCL and fromPCL functions
namespace pcl_conversions
{
	std::vector<pcl::PointIndices::Ptr>
		convertToPCLPointIndices(const std::vector<PCLIndicesMsg>& cluster_indices);
	std::vector<pcl::ModelCoefficients::Ptr>
		convertToPCLModelCoefficients(
				const std::vector<PCLModelCoefficientMsg>& coefficients);
	std::vector<PCLIndicesMsg>
		convertToROSPointIndices(
				const std::vector<pcl::PointIndices::Ptr> cluster_indices,
				const std_msgs::Header& header);
	std::vector<PCLIndicesMsg>
		convertToROSPointIndices(
				const std::vector<pcl::PointIndices> cluster_indices,
				const std_msgs::Header& header);
	std::vector<PCLModelCoefficientMsg>
		convertToROSModelCoefficients(
				const std::vector<pcl::ModelCoefficients::Ptr>& coefficients,
				const std_msgs::Header& header);
}
namespace tf
{
	// for eigen float
	void poseMsgToEigen(const geometry_msgs::Pose& msg, Eigen::Affine3f& eigen);
	void poseEigenToMsg(Eigen::Affine3f& eigen, geometry_msgs::Pose& msg);
	void transformMsgToEigen(const geometry_msgs::Transform& msg, Eigen::Affine3f& eigen);
	void transformEigenToMsg(Eigen::Affine3f& eigen, geometry_msgs::Transform& msg);
	void transformTFToEigen(const tf::Transform& t, Eigen::Affine3f& eigen);
	void transformEigenToTF(Eigen::Affine3f& eigen , tf::Transform& t);
}
#endif
