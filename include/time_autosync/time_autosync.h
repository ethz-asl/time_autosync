#ifndef TIME_AUTOSYNC_TIME_AUTOSYNC_H
#define TIME_AUTOSYNC_TIME_AUTOSYNC_H

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include "time_autosync/cdkf.h"

class TimeAutosync {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TimeAutosync(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:
  double calcAngleBetweenPointclouds(const pcl::PointCloud<pcl::PointXYZ>& prev_pointcloud,
                                const pcl::PointCloud<pcl::PointXYZ>& pointcloud);

  double calcAngleBetweenImages(const cv::Mat& prev_image,
                                const cv::Mat& image);

  void imuCallback(const sensor_msgs::ImuConstPtr& msg);

  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void setupCDKF();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport it_;

  ros::Subscriber imu_sub_;
  ros::Subscriber pointcloud_sub_;
  image_transport::Subscriber image_sub_;

  ros::Publisher delta_t_pub_;
  ros::Publisher offset_pub_;
  ros::Publisher pointcloud_pub_;
  image_transport::Publisher image_pub_;

  bool stamp_on_arrival_;
  double max_imu_data_age_s_;
  int delay_by_n_frames_;
  double focal_length_;
  bool use_pointcloud_;
  bool calc_offset_;

  std::unique_ptr<CDKF> cdkf_;

  IMUList imu_rotations_;
};

#endif  // TIME_AUTOSYNC_TIME_AUTOSYNC_H