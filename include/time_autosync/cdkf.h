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
    double inital_delta_t = 0.1;
    double inital_offset = 0.0;

    // inital noise values
    double inital_delta_t_sd = 0.1;
    double inital_offset_sd = 0.2;

    // measurement noise
    double timestamp_sd = 0.02;

    // process noise
    double delta_t_sd = 0.0001;
    double offset_sd = 0.0001;
  };

  CDKF(const Config& config) {
    state_.resize(kStateCoreSize, 1);
    accessS(state_, DELTA_T).array() = config.inital_delta_t;
    accessS(state_, OFFSET).array() = config.inital_offset;

    Eigen::MatrixXd inital_sd(kStateCoreSize, 1);
    accessS(inital_sd, DELTA_T).array() = config.inital_delta_t_sd;
    accessS(inital_sd, OFFSET).array() = config.inital_offset_sd;

    cov_.resize(kStateCoreSize, kStateCoreSize);
    cov_.setZero();
    cov_.diagonal() = inital_sd.array() * inital_sd.array();

    // set noise sd
    prediction_noise_sd_.resize(kPredictionNoiseSize, 1);
    accessS(prediction_noise_sd_, DELTA_T_NOISE).array() = config.delta_t_sd;
    accessS(prediction_noise_sd_, OFFSET_NOISE).array() = config.offset_sd;

    measurement_noise_sd_.resize(kMeasurementNoiseSize, 1);
    measurement_noise_sd_.array() = config.timestamp_sd;
  }

  void predictionUpdate() {
    ROS_WARN_STREAM("Initial State: \n" << state_.transpose());
    ROS_WARN_STREAM("Initial Cov: \n" << cov_);

    StateSigmaPoints sigma_points(state_, cov_, prediction_noise_sd_,
                                  CDKF::propergateState);

    sigma_points.calcEstimatedMean(&state_);
    sigma_points.calcEstimatedCov(&cov_);

    ROS_WARN_STREAM("Predicted State: \n" << state_.transpose());
    ROS_WARN_STREAM("Predicted Cov: \n" << cov_);
  }

  void measurementUpdate(const double measured_delta_t) {

    // convert tracked points to measurement
    Eigen::VectorXd real_measurement(kMeasurementElementSize);
    accessM(real_measurement, TIMESTAMP_DELTA).array() = measured_delta_t;

    // create sigma points
    MeasurementSigmaPoints sigma_points(state_, cov_, measurement_noise_sd_,
                                        CDKF::stateToMeasurementEstimate);

    // get mean and cov
    Eigen::VectorXd predicted_measurement;
    sigma_points.calcEstimatedMean(&predicted_measurement);
    Eigen::MatrixXd innovation;
    sigma_points.calcEstimatedCov(&innovation);

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
  }

  static void stateToMeasurementEstimate(
      const Eigen::VectorXd& input_state, const Eigen::VectorXd& noise,
      Eigen::Ref<Eigen::VectorXd> estimated_measurement) {
    accessM(estimated_measurement, TIMESTAMP_DELTA) =
        accessS(input_state, DELTA_T) + accessM(noise, TIMESTAMP_NOISE);
  }

  static void propergateState(const Eigen::VectorXd& noise,
                              Eigen::Ref<Eigen::VectorXd> current_state) {
    accessS(current_state, DELTA_T) += accessS(noise, DELTA_T_NOISE);
    accessS(current_state, OFFSET) += accessS(noise, OFFSET_NOISE);
  }

 private:
  Eigen::VectorXd state_;
  Eigen::MatrixXd cov_;

  Eigen::VectorXd prediction_noise_sd_;
  Eigen::VectorXd measurement_noise_sd_;
};

#endif  // STATE_DATA_TIME_AUTOSYNC_H
