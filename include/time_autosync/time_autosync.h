#ifndef TIME_AUTOSYNC_TIME_AUTOSYNC_H
#define TIME_AUTOSYNC_TIME_AUTOSYNC_H

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "time_autosync/cdkf.h"

class TimeAutosync {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TimeAutosync(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:
  double calcAngleBetweenImages(const cv::Mat& prev_image, const cv::Mat& image);

  void imuCallback(const sensor_msgs::ImuConstPtr& msg);

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void setupCDKF();

  static Eigen::Quaterniond getInterpolatedImuAngle(
      const AlignedList<std::pair<ros::Time, Eigen::Quaterniond>>&
          imu_rotations,
      const ros::Time& stamp);

  static double getImuAngleChange(
      const AlignedList<std::pair<ros::Time, Eigen::Quaterniond>>&
          imu_rotations,
      const ros::Time& start_stamp, const ros::Time& end_stamp);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport it_;

  ros::Subscriber imu_sub_;
  image_transport::Subscriber image_sub_;

  image_transport::Publisher image_pub_;

  bool verbose_;
  bool stamp_on_arrival_;

  double max_imu_data_age_s_;

  std::unique_ptr<CDKF> cdkf_;

  AlignedList<std::pair<ros::Time, Eigen::Quaterniond>> imu_rotations_;
};

#endif  // TIME_AUTOSYNC_TIME_AUTOSYNC_H