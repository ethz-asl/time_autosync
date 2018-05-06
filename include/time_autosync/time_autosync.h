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

  TimeAutosync(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

 private:

  double calcAngleFromImage(const cv::Mat &image);

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport it_;

  image_transport::Subscriber image_sub_;

  image_transport::Publisher image_pub_;

  bool verbose_;
  bool stamp_on_arrival_;

  double max_imu_data_age_s_;

  std::unique_ptr<CDKF> cdkf_;

  AlignedList<std::pair<ros::Time, Eigen::Quaterniond>> imu_rotations;
};

#endif  // TIME_AUTOSYNC_TIME_AUTOSYNC_H