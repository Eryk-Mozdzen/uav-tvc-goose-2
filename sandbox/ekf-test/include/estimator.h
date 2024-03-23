#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "ekf.h"

extern ekf_t ekf;
extern ekf_system_model_t rotation_model;
extern ekf_measurement_model_t accelerometer_model;
extern ekf_measurement_model_t magnetometer_model;

EKF_PREDICT_DEF(7, 3)
EKF_CORRECT_DEF(7, 3)

#endif
