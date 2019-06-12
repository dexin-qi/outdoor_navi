#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/point_cloud.h>

#include <std_msgs/Time.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <opac_navi/tick_tock.h>


#define __MOUDLENAME__ "\033[1;34m[lidar_correction]\033[0m "

namespace opac_navi
{

class OptimLidarNodelet : public nodelet::Nodelet
{

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  // ROS topics
  ros::Subscriber points_sub;
  std::string points_topic;
  
  ros::Publisher correct_cloud_pub;


public:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointXYZ  PointS;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OptimLidarNodelet() {}
  virtual ~OptimLidarNodelet() {}

  virtual void onInit()
  {
    ROS_INFO_STREAM(__MOUDLENAME__ << "initializing " << "...");
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    points_sub = nh.subscribe(points_topic, 64, &OptimLidarNodelet::cloud_callback, this);

    correct_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/optim_lidar/true_points", 64);

  }

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params()
  {
    points_topic = private_nh.param<std::string>("points_topic", "/rslidar_points");

  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    TickTock lidar_correction_tim;

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if (cloud->empty())
    {
      return;
    }

    pcl::PointCloud<PointT>::Ptr correct_cloud(new pcl::PointCloud<PointT>());
    for(PointT p: *cloud)
    {
      if (!pcl_isfinite (p.x) || !pcl_isfinite (p.y) || !pcl_isfinite (p.z))
        continue;
      PointT cr_p;
      cr_p.x = p.x/2.0;
      cr_p.y = p.y/2.0;
      cr_p.z = p.z/2.0;
      cr_p.intensity = p.intensity;

      correct_cloud->points.push_back(cr_p);
    }

    correct_cloud->header = cloud->header;
    correct_cloud_pub.publish(correct_cloud);

    double ms = lidar_correction_tim.tock();
    ROS_INFO_STREAM(__MOUDLENAME__ << "time(ms): " << ms);
  }

  /**
   * @brief plane_clip
   * @param src_cloud
   * @param plane
   * @param negative
   * @return
   */
  pcl::PointCloud<PointT>::Ptr plane_clip(const pcl::PointCloud<PointT>::Ptr &src_cloud, const Eigen::Vector4f &plane, bool negative) const
  {
    pcl::PlaneClipper3D<PointT> clipper(plane);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    clipper.clipPointCloud3D(*src_cloud, indices->indices);

    pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*dst_cloud);

    return dst_cloud;
  }
};

} // namespace opac_navi

PLUGINLIB_EXPORT_CLASS(opac_navi::OptimLidarNodelet, nodelet::Nodelet)
