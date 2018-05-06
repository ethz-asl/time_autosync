#include "time_autosync/time_autosync.h"

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/xfeatures2d.hpp>

TimeAutosync::TimeAutosync(const ros::NodeHandle &nh,
                           const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      it_(nh_private_),
      verbose_(false),
      stamp_on_arrival_(false) {
  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("stamp_on_arrival", stamp_on_arrival_, stamp_on_arrival_);

  UKF::Config config;

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

  ukf_ = std::unique_ptr<UKF>(new UKF(config));

  image_sub_ =
      it_.subscribe("input/image", 10, &TimeAutosync::imageCallback, this);

  image_pub_ = image_pub_ = it_.advertise("output/image", 10);
}

void TimeAutosync::calcAngleFromImage(const cv::Mat &image) {
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
    cv::arrowedLine(viz_image, tracked_prev_points, tracked_points,
                    cv::scalar(0, 1, 0));
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

  Eigen::Map<Matrix3d> E(cv_E.data());

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(E);

  Eigen::Matrix3d S, W;
  S.setZero();
  S.diagonal() = svd.singularValues();
  W << 0, -1, 0, 1, 0, 0, 0, 0, 1;

  Ra = svd.matrixU() * W * svd.matrixV().transpose();
  Rb = svd.matrixU() * W.t() * svd.matrixV().transpose();

  double angle =
      std::min(Eigen::AngleAxisd(Ra).angle(), Eigen::AngleAxisd(Rb).angle());

  return angle;
}

void TimeAutosync::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  ROS_ERROR("IMAGE CALLBACK");

  cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(msg, "mono8");
  calcAngleFromImage(image->image);

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
  ukf_->predictionUpdate();
  ROS_ERROR("MEASUREMENT UPDATE");
  ukf_->measurementUpdate(delta_t);
  ROS_ERROR("DONE");
}
