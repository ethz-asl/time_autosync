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
      max_imu_data_age_s_(1.0) {
  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("stamp_on_arrival", stamp_on_arrival_, stamp_on_arrival_);
  nh_private_.param("max_imu_data_age_s", max_imu_data_age_s_,
                    max_imu_data_age_s_);

  image_sub_ =
      it_.subscribe("input/image", 10, &TimeAutosync::imageCallback, this);

  imu_sub_ =
      nh_private_.subscribe("input/imu", 100, &TimeAutosync::imuCallback, this);

  image_pub_ = image_pub_ = it_.advertise("output/image", 10);

  myfile.open("/home/z/Desktop/test.csv");
}

void TimeAutosync::setupCDKF() {
  CDKF::Config config;

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
  constexpr int kMaxCorners = 1000;
  constexpr double kQualityLevel = 0.01;
  constexpr double kMinDistance = 10;

  std::vector<cv::Point2f> prev_points;
  cv::goodFeaturesToTrack(prev_image, prev_points, kMaxCorners, kQualityLevel,
                          kMinDistance);

  if (prev_points.size() == 0) {
    ROS_ERROR("prev points empty wtf");
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

  cv::Mat viz_image;
  cv::cvtColor(prev_image, viz_image, cv::COLOR_GRAY2BGR);

  for (size_t i = 0; i < tracked_points.size(); ++i) {
    cv::arrowedLine(viz_image, tracked_prev_points[i], tracked_points[i],
                    cv::Scalar(0, 255, 0));
  }

  cv::namedWindow("Tracked Points", cv::WINDOW_AUTOSIZE);
  cv::imshow("Tracked Points", viz_image);
  cv::waitKey(1);

  // noramilze values

  /*constexpr double kFocalLength = 460.0;
  // close enough for most cameras given the low level of accuracy needed
  const cv::Point2f offset(image.cols / 2.0, image.rows / 2.0);

  for (size_t i = 0; i < tracked_points.size(); ++i) {
    tracked_prev_points[i] = (tracked_prev_points[i] - offset) / kFocalLength;
    tracked_points[i] = (tracked_points[i] - offset) / kFocalLength;
  }*/

  constexpr double kMaxEpipoleDistance = 1e-3;
  constexpr double kInlierProbability = 0.99;

  std::vector<uint8_t> inliers;
  cv::Mat cv_F =
      cv::findFundamentalMat(tracked_prev_points, tracked_points, cv::FM_LMEDS,
                             kMaxEpipoleDistance, kInlierProbability, inliers);

  Eigen::Matrix3d E, W, K;

  K << 458.654, 0.0, 367.215, 0.0, 457.296, 248.375, 0.0, 0.0, 1.0;

  cv::cv2eigen(cv_F, E);

  E = K.transpose() * E * K;

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

Eigen::Quaterniond TimeAutosync::getInterpolatedImuAngle(
    const AlignedList<std::pair<ros::Time, Eigen::Quaterniond>>& imu_rotations,
    const ros::Time& stamp) {
  Eigen::Quaterniond angle;

  AlignedList<std::pair<ros::Time, Eigen::Quaterniond>>::const_iterator prev_it,
      it;
  prev_it = imu_rotations.begin();

  // find location of starting stamp
  for (it = imu_rotations.begin();
       (std::next(it) != imu_rotations.end()) && (it->first < stamp);
       prev_it = it++)
    ;

  // interpolate to get angle
  if (prev_it->first == it->first) {
    angle = it->second;
  } else {
    const double delta_t = (it->first - prev_it->first).toSec();
    double w = (it->first - stamp).toSec() / delta_t;

    // don't extrapolate
    if (w < 0.0) {
      ROS_WARN("Trying to get Imu data from too far in the past");
      w = 0.0;
    } else if (w > 1.0) {
      ROS_WARN("Trying to get Imu data from too far in the future");
      w = 1.0;
    }

    angle = prev_it->second.slerp(w, it->second);
  }

  return angle;
}

double TimeAutosync::getImuAngleChange(
    const AlignedList<std::pair<ros::Time, Eigen::Quaterniond>>& imu_rotations,
    const ros::Time& start_stamp, const ros::Time& end_stamp) {
  Eigen::Quaterniond start_angle =
      getInterpolatedImuAngle(imu_rotations, start_stamp);
  Eigen::Quaterniond end_angle =
      getInterpolatedImuAngle(imu_rotations, end_stamp);
  Eigen::AngleAxisd diff_angle(start_angle.inverse() * end_angle);

  return diff_angle.angle();
}

void TimeAutosync::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

  ros::Time stamp;
  if (stamp_on_arrival_) {
    stamp = ros::Time::now();
  } else {
    stamp = msg->header.stamp;
  }

  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg, "mono8");
  image->header.stamp = stamp;

  // put a 1 image delay in the estimation to allow for the possibility that the
  // image is coming in before the imu messages
  static cv_bridge::CvImage prev_image;
  static cv_bridge::CvImage prev_prev_image;
  static size_t seen_atleast_n_images = 0;

  ROS_ERROR_STREAM("IMAGE CALLBACK " << seen_atleast_n_images);

  if (seen_atleast_n_images == 0) {
    seen_atleast_n_images = 1;
    prev_image = *image;
    return;
  } else if (seen_atleast_n_images == 1) {
    seen_atleast_n_images = 2;
    prev_prev_image = prev_image;
    prev_image = *image;

    // leave creating filter as late as possible as if we are playing off a bag
    // we need to grab its clock
    setupCDKF();
    return;
  }

  if (imu_rotations_.size() < 2) {
    return;
  }

  double image_angle =
      calcAngleBetweenImages(prev_prev_image.image, prev_image.image);
  double imu_angle = getImuAngleChange(
      imu_rotations_, prev_prev_image.header.stamp, prev_image.header.stamp);

  myfile << image_angle << "," << imu_angle << "\n";

  prev_prev_image = prev_image;
  prev_image = *image;

  /*ros::Time stamp;
  if (stamp_on_arrival_) {
    stamp = ros::Time::now();
  } else {
    stamp = msg->header.stamp;
  }

  if (!initalized) {
    prev_stamp = stamp;
    initalized = true;
    return;
  }

  if (prev_stamp >= stamp) {
    ROS_ERROR(
        "The timestamps just went back in time. If your timings are really "
        "this messed up it may be better to set stamp_on_arrival to true");
  }*/

  // actually run filter
  ROS_ERROR("IMU UPDATE");
  cdkf_->predictionUpdate(prev_image.header.stamp);
  ROS_ERROR("MEASUREMENT UPDATE");
  cdkf_->measurementUpdate(prev_prev_image.header.stamp,
                           prev_image.header.stamp);
  ROS_ERROR("DONE");
}
