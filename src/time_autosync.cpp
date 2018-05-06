#include "time_autosync/time_autosync.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <fstream>
#include <iostream>

std::ofstream myfile;

TimeAutosync::TimeAutosync(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      it_(nh_private_),
      verbose_(false),
      stamp_on_arrival_(false),
      max_imu_data_age_s_(2.0),
      delay_by_n_frames_(2),
      focal_length_(460.0),
      calc_offset(false) {
  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("stamp_on_arrival", stamp_on_arrival_, stamp_on_arrival_);
  nh_private_.param("max_imu_data_age_s", max_imu_data_age_s_,
                    max_imu_data_age_s_);
  nh_private_.param("delay_by_n_frames", delay_by_n_frames_,
                    delay_by_n_frames_);
  nh_private_.param("focal_length", focal_length_, focal_length_);
  nh_private_.param("calc_offset", calc_offset, calc_offset);

  setupCDKF();

  image_sub_ =
      it_.subscribe("input/image", 10, &TimeAutosync::imageCallback, this);

  imu_sub_ =
      nh_private_.subscribe("input/imu", 100, &TimeAutosync::imuCallback, this);

  image_pub_ = image_pub_ = it_.advertise("output/image", 10);

  myfile.open("/home/z/Desktop/test.csv");
}

void TimeAutosync::setupCDKF() {
  CDKF::Config config;

  nh_private_.param("verbose", config.verbose, config.verbose);
  nh_private_.param("mah_threshold", config.mah_threshold,
                    config.mah_threshold);

  nh_private_.param("inital_delta_t", config.inital_delta_t,
                    config.inital_delta_t);
  nh_private_.param("inital_offset", config.inital_offset,
                    config.inital_offset);

  nh_private_.param("inital_delta_t_sd", config.inital_delta_t_sd,
                    config.inital_delta_t_sd);
  nh_private_.param("inital_offset_sd", config.inital_offset_sd,
                    config.inital_offset_sd);

  nh_private_.param("timestamp_sd", config.timestamp_sd, config.timestamp_sd);

  nh_private_.param("delta_t_sd", config.delta_t_sd, config.delta_t_sd);
  nh_private_.param("offset_sd", config.offset_sd, config.offset_sd);

  cdkf_ = std::unique_ptr<CDKF>(new CDKF(config));
}

double TimeAutosync::calcAngleBetweenImages(const cv::Mat& prev_image,
                                            const cv::Mat& image) {
  constexpr int kMaxCorners = 50;
  constexpr double kQualityLevel = 0.01;
  constexpr double kMinDistance = 10;

  std::vector<cv::Point2f> prev_points;
  cv::goodFeaturesToTrack(prev_image, prev_points, kMaxCorners, kQualityLevel,
                          kMinDistance);

  if (prev_points.size() == 0) {
    ROS_ERROR("Tracking has failed cannot calculate angle");
    return 0.0;
  }

  std::vector<cv::Point2f> points;
  std::vector<uint8_t> valid;
  std::vector<float> err;
  cv::calcOpticalFlowPyrLK(prev_image, image, prev_points, points, valid, err);

  std::vector<cv::Point2f> tracked_prev_points, tracked_points;
  for (size_t i = 0; i < prev_points.size(); ++i) {
    if (valid[i]) {
      tracked_prev_points.push_back(prev_points[i]);
      tracked_points.push_back(points[i]);
    }
  }

  /*cv::Mat viz_image;
  cv::cvtColor(prev_image, viz_image, cv::COLOR_GRAY2BGR);

  for (size_t i = 0; i < tracked_points.size(); ++i) {
    cv::arrowedLine(viz_image, tracked_prev_points[i], tracked_points[i],
                    cv::Scalar(0, 255, 0));
  }

  cv::namedWindow("Tracked Points", cv::WINDOW_AUTOSIZE);
  cv::imshow("Tracked Points", viz_image);
  cv::waitKey(1);*/

  // close enough for most cameras given the low level of accuracy needed
  const cv::Point2f offset(image.cols / 2.0, image.rows / 2.0);

  for (size_t i = 0; i < tracked_points.size(); ++i) {
    tracked_prev_points[i] = (tracked_prev_points[i] - offset) / focal_length_;
    tracked_points[i] = (tracked_points[i] - offset) / focal_length_;
  }

  constexpr double kMaxEpipoleDistance = 1e-3;
  constexpr double kInlierProbability = 0.99;

  std::vector<uint8_t> inliers;
  cv::Mat cv_F =
      cv::findFundamentalMat(tracked_prev_points, tracked_points, cv::FM_LMEDS,
                             kMaxEpipoleDistance, kInlierProbability, inliers);

  Eigen::Matrix3d E, W;

  cv::cv2eigen(cv_F, E);

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      E, Eigen::ComputeThinU | Eigen::ComputeThinV);

  W << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix3d Ra = svd.matrixU() * W * svd.matrixV().transpose();
  Eigen::Matrix3d Rb =
      svd.matrixU() * W.transpose() * svd.matrixV().transpose();

  double angle =
      std::min(Eigen::AngleAxisd(Ra).angle(), Eigen::AngleAxisd(Rb).angle());

  return angle;
}

void TimeAutosync::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  static sensor_msgs::Imu prev_msg;
  static bool first_msg = true;

  if (first_msg) {
    first_msg = false;
    imu_rotations_.emplace_back(msg->header.stamp,
                                Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
    prev_msg = *msg;
    return;
  }

  if (prev_msg.header.stamp >= msg->header.stamp) {
    ROS_WARN(
        "Your imu messages are not monotonically increasing, expect garbage "
        "results.");
  }

  // integrate imu reading
  double half_delta_t =
      (msg->header.stamp - prev_msg.header.stamp).toSec() / 2.0;

  Eigen::Quaterniond delta_angle =
      Eigen::AngleAxisd(half_delta_t * (msg->angular_velocity.x +
                                        prev_msg.angular_velocity.x),
                        Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(half_delta_t * (msg->angular_velocity.y +
                                        prev_msg.angular_velocity.y),
                        Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(half_delta_t * (msg->angular_velocity.z +
                                        prev_msg.angular_velocity.z),
                        Eigen::Vector3d::UnitZ());

  imu_rotations_.emplace_back(
      prev_msg.header.stamp + ros::Duration(half_delta_t),
      imu_rotations_.back().second * delta_angle);
  imu_rotations_.back().second.normalize();

  // clear old data
  while ((imu_rotations_.back().first - imu_rotations_.front().first).toSec() >
         max_imu_data_age_s_) {
    imu_rotations_.pop_front();
  }

  prev_msg = *msg;
}

void TimeAutosync::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ros::Time stamp;
  if (stamp_on_arrival_) {
    stamp = ros::Time::now();
  } else {
    stamp = msg->header.stamp + ros::Duration(0.2);
  }

  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg, "mono8");
  image->header.stamp = stamp;

  // delay by a few messages to ensure IMU messages span needed range
  static std::list<cv_bridge::CvImage> images;
  images.push_back(*image);

  if (images.size() < delay_by_n_frames_) {
    cdkf_->rezeroTimestamps(images.front().header.stamp, true);
    return;
  }

  if (calc_offset && (imu_rotations_.size() < 2)) {
    return;
  }

  double image_angle = 0.0;
  if (calc_offset) {
    image_angle = calcAngleBetweenImages(images.begin()->image,
                                         std::next(images.begin())->image);
  }

  // actually run filter
  cdkf_->predictionUpdate(std::next(images.begin())->header.stamp);
  cdkf_->measurementUpdate(images.begin()->header.stamp,
                           std::next(images.begin())->header.stamp, image_angle,
                           imu_rotations_, calc_offset);

  images.pop_front();
}
