#include "time_autosync/time_autosync.h"

#include <pcl/filters/random_sample.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

TimeAutosync::TimeAutosync(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      it_(nh_private_),
      stamp_on_arrival_(false),
      max_imu_data_age_s_(2.0),
      delay_by_n_frames_(5),
      focal_length_(460.0),
      use_pointcloud_(false),
      calc_offset_(true) {
  nh_private_.param("stamp_on_arrival", stamp_on_arrival_, stamp_on_arrival_);
  nh_private_.param("max_imu_data_age_s", max_imu_data_age_s_,
                    max_imu_data_age_s_);
  nh_private_.param("delay_by_n_frames", delay_by_n_frames_,
                    delay_by_n_frames_);
  nh_private_.param("focal_length", focal_length_, focal_length_);
  nh_private_.param("use_pointcloud", use_pointcloud_, use_pointcloud_);
  nh_private_.param("calc_offset", calc_offset_, calc_offset_);

  setupCDKF();

  constexpr int kImageQueueSize = 10;
  constexpr int kPointcloudQueueSize = 10;
  constexpr int kImuQueueSize = 100;
  constexpr int kFloatQueueSize = 100;

  if (use_pointcloud_) {
    pointcloud_sub_ =
        nh_private_.subscribe("input/pointcloud", kPointcloudQueueSize,
                              &TimeAutosync::pointcloudCallback, this);
    pointcloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>(
        "output/pointcloud", kPointcloudQueueSize);
  } else {
    image_sub_ = it_.subscribe("input/image", kImageQueueSize,
                               &TimeAutosync::imageCallback, this);
    image_pub_ = image_pub_ = it_.advertise("output/image", kImageQueueSize);
  }

  imu_sub_ = nh_private_.subscribe("input/imu", kImuQueueSize,
                                   &TimeAutosync::imuCallback, this);

  delta_t_pub_ =
      nh_private_.advertise<std_msgs::Float64>("delta_t", kFloatQueueSize);

  if (calc_offset_) {
    offset_pub_ =
        nh_private_.advertise<std_msgs::Float64>("offset", kFloatQueueSize);
  }
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

double TimeAutosync::calcAngleBetweenPointclouds(
    const pcl::PointCloud<pcl::PointXYZ>& prev_pointcloud,
    const pcl::PointCloud<pcl::PointXYZ>& pointcloud) {
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  pcl::PointCloud<pcl::PointXYZ>::Ptr prev_pointcloud_sampled(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_sampled(
      new pcl::PointCloud<pcl::PointXYZ>);

  // shared_pointers needed by icp, no-op destructor to prevent them being
  // cleaned up after use
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_pointcloud_ptr(
      &prev_pointcloud, [](const pcl::PointCloud<pcl::PointXYZ>*) {});
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud_ptr(
      &pointcloud, [](const pcl::PointCloud<pcl::PointXYZ>*) {});

  constexpr int kMaxSamples = 2000;
  pcl::RandomSample<pcl::PointXYZ> sample(false);
  sample.setSample(kMaxSamples);

  sample.setInputCloud(prev_pointcloud_ptr);
  sample.filter(*prev_pointcloud_sampled);

  sample.setInputCloud(pointcloud_ptr);
  sample.filter(*pointcloud_sampled);

  icp.setInputSource(prev_pointcloud_sampled);
  icp.setInputTarget(pointcloud_sampled);

  pcl::PointCloud<pcl::PointXYZ> final;
  icp.align(final);

  Eigen::Matrix4f tform = icp.getFinalTransformation();
  double angle =
      Eigen::AngleAxisd(tform.topLeftCorner<3, 3>().cast<double>()).angle();

  return angle;
}

double TimeAutosync::calcAngleBetweenImages(const cv::Mat& prev_image,
                                            const cv::Mat& image) {
  constexpr int kMaxCorners = 100;
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

void TimeAutosync::pointcloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  ros::Time stamp;
  if (stamp_on_arrival_) {
    stamp = ros::Time::now();
  } else {
    stamp = msg->header.stamp;
  }

  static std::list<std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>>>
      pointclouds;
  std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>> pointcloud;
  pcl::fromROSMsg(*msg, pointcloud.second);

  // fire the pointcloud back out with minimal lag
  if (pointclouds.size() >= (delay_by_n_frames_ - 1)) {
    std_msgs::Float64 delta_t, offset;
    cdkf_->getSyncedTimestamp(stamp, &(pointcloud.first), &(delta_t.data),
                              &(offset.data));

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(pointcloud.second, msg_out);

    pointcloud_pub_.publish(msg_out);
    delta_t_pub_.publish(delta_t);
    if (calc_offset_) {
      offset_pub_.publish(offset);
    }
  }

  pointcloud.first = stamp;

  // delay by a few messages to ensure IMU messages span needed range
  pointclouds.push_back(pointcloud);

  if (pointclouds.size() < delay_by_n_frames_) {
    cdkf_->rezeroTimestamps(pointclouds.front().first, true);
    return;
  }

  if (calc_offset_ && (imu_rotations_.size() < 2)) {
    return;
  }

  double pointcloud_angle = 0.0;
  if (calc_offset_) {
    pointcloud_angle = calcAngleBetweenPointclouds(
        pointclouds.begin()->second, std::next(pointclouds.begin())->second);
  }

  // actually run filter
  cdkf_->predictionUpdate(std::next(pointclouds.begin())->first);
  cdkf_->measurementUpdate(pointclouds.begin()->first,
                           std::next(pointclouds.begin())->first,
                           pointcloud_angle, imu_rotations_, calc_offset_);

  pointclouds.pop_front();
}

void TimeAutosync::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ros::Time stamp;
  if (stamp_on_arrival_) {
    stamp = ros::Time::now();
  } else {
    stamp = msg->header.stamp;
  }

  static std::list<cv_bridge::CvImage> images;
  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg, "mono8");

  // fire the image back out with minimal lag
  if (images.size() >= (delay_by_n_frames_ - 1)) {
    std_msgs::Float64 delta_t, offset;
    cdkf_->getSyncedTimestamp(stamp, &(image->header.stamp), &(delta_t.data),
                              &(offset.data));
    image_pub_.publish(image->toImageMsg());
    delta_t_pub_.publish(delta_t);
    if (calc_offset_) {
      offset_pub_.publish(offset);
    }
  }

  image->header.stamp = stamp;

  // delay by a few messages to ensure IMU messages span needed range
  images.push_back(*image);

  if (images.size() < delay_by_n_frames_) {
    cdkf_->rezeroTimestamps(images.front().header.stamp, true);
    return;
  }

  if (calc_offset_ && (imu_rotations_.size() < 2)) {
    return;
  }

  double image_angle = 0.0;
  if (calc_offset_) {
    image_angle = calcAngleBetweenImages(images.begin()->image,
                                         std::next(images.begin())->image);
  }

  // actually run filter
  cdkf_->predictionUpdate(std::next(images.begin())->header.stamp);
  cdkf_->measurementUpdate(images.begin()->header.stamp,
                           std::next(images.begin())->header.stamp, image_angle,
                           imu_rotations_, calc_offset_);

  images.pop_front();
}
