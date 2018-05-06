#include "time_autosync/time_autosync.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

TimeAutosync::TimeAutosync(const ros::NodeHandle &nh,
                           const ros::NodeHandle &nh_private)
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

  image_sub_ =
      it_.subscribe("input/image", 10, &TimeAutosync::imageCallback, this);

  image_pub_ = image_pub_ = it_.advertise("output/image", 10);
}

double TimeAutosync::calcAngleFromImage(const cv::Mat &image) {
  static cv::Mat prev_image;

  if (prev_image.empty()) {
    prev_image = image;
    return 0.0;
  }

  constexpr int kMaxCorners = 100;
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

  constexpr double kFocalLength = 460.0;
  // close enough for most cameras given the low level of accuracy needed
  const cv::Point2f offset(image.cols / 2.0, image.rows / 2.0);

  for (size_t i = 0; i < tracked_points.size(); ++i) {
    tracked_prev_points[i] = (tracked_prev_points[i] - offset) / kFocalLength;
    tracked_points[i] = (tracked_points[i] - offset) / kFocalLength;
  }

  constexpr double kMaxEpipoleDistance = 1.0;
  constexpr double kInlierProbability = 0.99;

  std::vector<uint8_t> inliers;
  cv::Mat cv_E =
      cv::findFundamentalMat(tracked_prev_points, tracked_points, cv::FM_LMEDS,
                             kMaxEpipoleDistance, kInlierProbability, inliers);

  prev_image = image;

  Eigen::Matrix3d E, W, K;

  cv::cv2eigen(cv_E, E);

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

void TimeAutosync::imuCallback(const sensor_msgs::ImuConstPtr &msg) {
  ROS_ERROR("IMU CALLBACK");
  static sensor_msgs::Imu prev_msg;
  static bool first_msg = true;

  if (first_msg) {
    first_msg = false;
    imu_rotations_.emplace_back(msg->header.stamp,
                                Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
    return;
  }

  if (prev_msg.header.stamp >= msg->header.stamp) {
    ROS_WARN(
        "Your imu messages are not monotonically increasing, expect garbage "
        "results.");
  }

  ros::Duration half_delta_t =
      (msg->header.stamp - prev_msg.header.stamp) / 2.0;

  Eigen::Quaterniond delta_angle =
      AngleAxisf(half_delta_t.toSec() * (msg->angular_velocity.x() +
                                         prev_msg.angular_velocity.x()),
                 Vector3f::UnitX()) *
      AngleAxisf(half_delta_t.toSec() * (msg->angular_velocity.y() +
                                         prev_msg.angular_velocity.y()),
                 Vector3f::UnitY()) *
      AngleAxisf(half_delta_t.toSec() * (msg->angular_velocity.z() +
                                         prev_msg.angular_velocity.z()),
                 Vector3f::UnitZ());

  imu_msgs_.emplace_back(prev_msg.header.stamp + half_delta_t,
                         imu_msgs_.back() * delta_angle);
  imu_msgs_.back().second.normalize();

  while ((imu_msgs_.back().first - imu_msgs_.front().first).toSec() >
         max_imu_data_age_s_) {
    imu_msgs_.pop_front();
  }
}

void TimeAutosync::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImageConstPtr image = cv_bridge::toCvCopy(msg, "mono8");

  // put a 1 image delay in the estimation to allow for the possibility that the
  // image is coming in before the imu messages
  static cv_bridge::CvImage prev_image;
  static bool first_msg = true;

  if (first_msg) {
    first_msg = false;
    prev_image = *image;
    return;
  }

  ROS_ERROR_STREAM("angle: " << calcAngleFromImage(prev_image.image));

  prev_image = *image;

  /*
  ros::Time stamp;

  static ros::Time prev_stamp;
  static bool initalized = false;

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
  }

  double delta_t = (stamp - prev_stamp).toSec();
  prev_stamp = stamp;

  // actually run filter
  ROS_ERROR("IMU UPDATE");
  cdkf_->predictionUpdate();
  ROS_ERROR("MEASUREMENT UPDATE");
  cdkf_->measurementUpdate(delta_t);
  ROS_ERROR("DONE");*/
}
