#ifndef STATE_DATA_TIME_AUTOSYNC_H
#define STATE_DATA_TIME_AUTOSYNC_H

#include <Eigen/Eigen>

#include "time_autosync/common.h"

enum StateElements : size_t {
  DELTA_T,
  OFFSET,
  NUM_STATE_ELEMENTS
};

enum PredictionNoiseElements : size_t {
  DELTA_T_NOISE,
  OFFSET_NOISE,
  NUM_PREDICTION_NOISE_ELEMENTS
};

enum MeasurementNoiseElements : size_t {
  TIMESTAMP_NOISE,
  NUM_MEASUREMENT_NOISE_ELEMENTS
};

enum MeasurementElements : size_t { TIMESTAMP_DELTA, NUM_MEASUREMENT_ELEMENTS };

constexpr size_t kStateElementSize = 1;
constexpr size_t kMeasurementElementSize = 1;

constexpr size_t kStateCoreSize = kStateElementSize * NUM_STATE_ELEMENTS;
constexpr size_t kMeasurementSize = kMeasurementElementSize * NUM_MEASUREMENT_ELEMENTS;

constexpr size_t kPredictionNoiseSize =
    kStateElementSize * NUM_PREDICTION_NOISE_ELEMENTS;
constexpr size_t kMeasurementNoiseSize =
    kMeasurementElementSize * NUM_MEASUREMENT_NOISE_ELEMENTS;

#endif  // STATE_DATA_TIME_AUTOSYNC_H
