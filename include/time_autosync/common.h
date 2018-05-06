#ifndef COMMON_AUTO_TIMESYNC_H
#define COMMON_AUTO_TIMESYNC_H

#include <list>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Eigen>

// Aligned Eigen containers
template <typename Type>
using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type>>;
template <typename Type>
using AlignedList = std::list<Type, Eigen::aligned_allocator<Type>>;

using IMUList = AlignedList<std::pair<ros::Time, Eigen::Quaterniond>>;

Eigen::Quaterniond getInterpolatedImuAngle(const IMUList& imu_rotations,
                                           const ros::Time& stamp) {

  ros::Time now = ros::Time::now();
  ROS_ERROR_STREAM("IMU   now: " << now.sec << "." << std::setfill('0') << std::setw(9) << now.nsec);
  ROS_ERROR_STREAM("IMU start: " << imu_rotations.front().first.sec << "." << std::setfill('0') << std::setw(9) << imu_rotations.front().first.nsec);
  ROS_ERROR_STREAM("IMU end  : " << imu_rotations.back().first.sec << "." << std::setfill('0') << std::setw(9) << imu_rotations.back().first.nsec);
  ROS_ERROR_STREAM("stamp    : " << stamp.sec << "." << std::setfill('0') << std::setw(9) << stamp.nsec);

  IMUList::const_iterator prev_it, it;
  prev_it = imu_rotations.begin();

  // find location of starting stamp
  for (it = imu_rotations.begin();
       ((std::next(it) != imu_rotations.end()) && ((stamp - it->first).toSec() > 0.0));
       prev_it = it++);

  ROS_ERROR_STREAM("ptamp    : " << prev_it->first.sec << "." << std::setfill('0') << std::setw(9) << prev_it->first.nsec);
  ROS_ERROR_STREAM("stamp    : " << it->first.sec << "." << std::setfill('0') << std::setw(9) << it->first.nsec);
  // interpolate to get angle
  Eigen::Quaterniond angle;
  if (prev_it->first == it->first) {
    angle = it->second;
  } else {
    const double delta_t = (it->first - prev_it->first).toSec();
    double w = (stamp - prev_it->first).toSec() / delta_t;

    // don't extrapolate
    ROS_ERROR_STREAM("W: " << w);
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

double getImuAngleChange(const IMUList& imu_rotations,
                         const ros::Time& start_stamp,
                         const ros::Time& end_stamp) {
  Eigen::Quaterniond start_angle =
      getInterpolatedImuAngle(imu_rotations, start_stamp);
  Eigen::Quaterniond end_angle =
      getInterpolatedImuAngle(imu_rotations, end_stamp);
  Eigen::AngleAxisd diff_angle(start_angle.inverse() * end_angle);

  return diff_angle.angle();
}

#endif  // COMMON_AUTO_TIMESYNC_H
