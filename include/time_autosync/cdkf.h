#ifndef CDKF_TIME_AUTOSYNC_H
#define CDKF_TIME_AUTOSYNC_H

#include <Eigen/Eigen>

#include "time_autosync/common.h"
#include "time_autosync/sigma_points.h"
#include "time_autosync/state_accessors.h"
#include "time_autosync/state_data.h"

class CDKF {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // inital values
    double inital_delta_t = 0.05;
    double inital_offset = 0.0;

    // inital noise values
    double inital_timestamp_sd = 0.5;
    double inital_delta_t_sd = 0.1;
    double inital_offset_sd = 0.2;

    // measurement noise
    double timestamp_sd = 0.02;
    double angular_velocity_sd = 0.02;

    // process noise
    double delta_t_sd = 0.0001;
    double offset_sd = 0.0001;
  };

  CDKF(const Config& config) {
    state_.resize(kStateSize, 1);
    accessS(state_, DELTA_T).array() = config.inital_delta_t;
    accessS(state_, OFFSET).array() = config.inital_offset;

    Eigen::MatrixXd inital_sd(kStateSize, 1);
    accessS(inital_sd, STATE_TIMESTAMP).array() = config.inital_timestamp_sd;
    accessS(inital_sd, DELTA_T).array() = config.inital_delta_t_sd;
    accessS(inital_sd, OFFSET).array() = config.inital_offset_sd;

    cov_.resize(kStateSize, kStateSize);
    cov_.setZero();
    cov_.diagonal() = inital_sd.array() * inital_sd.array();

    // set noise sd
    prediction_noise_sd_.resize(kStateSize, 1);
    accessS(prediction_noise_sd_, DELTA_T).array() = config.delta_t_sd;
    accessS(prediction_noise_sd_, OFFSET).array() = config.offset_sd;

    measurement_noise_sd_.resize(kMeasurementSize, 1);
    accessM(measurement_noise_sd_, MEASURED_TIMESTAMP).array() =
        config.timestamp_sd;
    /*accessM(measurement_noise_sd_, ANGULAR_VELOCITY_NOISE).array() =
        config.angular_velocity_sd;*/

    zero_timestamp_ = ros::Time::now();
  }

  // make all stored timestamps relative to this one, called periodically to
  // prevent loss in precision
  void rezeroTimestamps() {
    ros::Time new_zero_timestamp = ros::Time::now();
    double time_diff = (new_zero_timestamp - zero_timestamp_).toSec();
    accessS(state_, STATE_TIMESTAMP).array() -= time_diff;

    zero_timestamp_ = new_zero_timestamp;
  }

  // sync the measured timestamp based on the current filter state
  void getSyncTimestamp(const ros::Time& received_timestamp,
                        ros::Time* synced_timestamp) {
    *synced_timestamp =
        zero_timestamp_ + ros::Duration(accessS(state_, STATE_TIMESTAMP)[0]);

    double delta_t = accessS(state_, DELTA_T)[0];

    // account for sync being some frames behind
    int num_frames =
        std::round((received_timestamp - *synced_timestamp).toSec() / delta_t);

    if ((num_frames > 2) || (num_frames < 0)) {
      ROS_WARN_STREAM("Timesync is now off by "
                      << num_frames
                      << " frames, something must be going horribly wrong");
    }
    *synced_timestamp += ros::Duration(num_frames * delta_t);

    // finally account for the offset
    *synced_timestamp += ros::Duration(accessS(state_, OFFSET)[0]);
  }

  void predictionUpdate(const ros::Time& received_timestamp) {
    ROS_WARN_STREAM("Initial State: \n" << state_.transpose());
    ROS_WARN_STREAM("Initial Cov: \n" << cov_);

    StateSigmaPoints sigma_points(state_, cov_, prediction_noise_sd_,
                                  CDKF::propergateState,
                                  received_timestamp - zero_timestamp_);

    sigma_points.calcEstimatedMean(&state_);
    sigma_points.calcEstimatedCov(&cov_);

    ROS_WARN_STREAM("Predicted State: \n" << state_.transpose());
    ROS_WARN_STREAM("Predicted Cov: \n" << cov_);
  }

  void measurementUpdate(
      const ros::Time& prev_stamp, const ros::Time& current_stamp) {
    // convert tracked points to measurement
    Eigen::VectorXd real_measurement(kMeasurementElementSize);
    accessM(real_measurement, MEASURED_TIMESTAMP).array() =
        (current_stamp - zero_timestamp_).toSec();

    ROS_WARN_STREAM("measured " << real_measurement.transpose());

    // create sigma points
    MeasurementSigmaPoints sigma_points(state_, cov_, measurement_noise_sd_,
                                        CDKF::stateToMeasurementEstimate);

    // get mean and cov
    Eigen::VectorXd predicted_measurement;
    sigma_points.calcEstimatedMean(&predicted_measurement);
    Eigen::MatrixXd innovation;
    sigma_points.calcEstimatedCov(&innovation);

    ROS_WARN_STREAM("predicted " << predicted_measurement.transpose());

    // calc mah distance
    Eigen::VectorXd diff = real_measurement - predicted_measurement;
    double mah_dist = std::sqrt(diff.transpose() * innovation.inverse() * diff);
    if (mah_dist > 10) {
      ROS_ERROR("Mah triggered");
    }

    Eigen::MatrixXd cross_cov;
    sigma_points.calcEstimatedCrossCov(&cross_cov);

    Eigen::MatrixXd gain = cross_cov * innovation.inverse();

    const Eigen::VectorXd state_diff =
        gain * (real_measurement - predicted_measurement);

    state_ += state_diff;

    cov_ -= gain * innovation * gain.transpose();

    ROS_WARN_STREAM("Updated State: \n" << state_.transpose());
    ROS_WARN_STREAM("Updated Cov: \n" << cov_);

    // guard against precision issues
    constexpr double kMaxTime = 10000.0;
    if (accessS(state_, STATE_TIMESTAMP)[0] > kMaxTime) {
      rezeroTimestamps();
    }
  }

  static void stateToMeasurementEstimate(
      const Eigen::VectorXd& input_state, const Eigen::VectorXd& noise,
      Eigen::Ref<Eigen::VectorXd> estimated_measurement) {
    accessM(estimated_measurement, MEASURED_TIMESTAMP) =
        accessS(input_state, STATE_TIMESTAMP) + accessM(noise, STATE_TIMESTAMP);
  }

  static void propergateState(const ros::Duration& zeroed_receive_timestamp,
                              const Eigen::VectorXd& noise,
                              Eigen::Ref<Eigen::VectorXd> current_state) {
    // work out how many frames to go forward to guard against drops
    int num_frames = std::round((zeroed_receive_timestamp.toSec() -
                                 accessS(current_state, STATE_TIMESTAMP)[0]) /
                                accessS(current_state, DELTA_T)[0]);
    if (num_frames < 1) {
      num_frames = 1;
    }

    accessS(current_state, DELTA_T) += accessS(noise, DELTA_T);
    accessS(current_state, OFFSET) += accessS(noise, OFFSET);

    accessS(current_state, STATE_TIMESTAMP) +=
        accessS(current_state, DELTA_T) * num_frames;
  }

 private:
  ros::Time zero_timestamp_;

  Eigen::VectorXd state_;
  Eigen::MatrixXd cov_;

  Eigen::VectorXd prediction_noise_sd_;
  Eigen::VectorXd measurement_noise_sd_;
};

#endif  // STATE_DATA_TIME_AUTOSYNC_H
