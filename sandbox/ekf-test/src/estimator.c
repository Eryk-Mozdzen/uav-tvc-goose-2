#include "ekf.h"

static float x_data[7] = {1, 0, 0, 0, 0, 0, 0};

static float P_data[7*7] = {
    1, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 1
};

ekf_t ekf = {
    .x.numRows = 7,
    .x.numCols = 1,
    .x.pData = x_data,
    .P.numRows = 7,
    .P.numCols = 7,
    .P.pData = P_data
};

static void rotation_f(const arm_matrix_instance_f32 *x, const arm_matrix_instance_f32 *u, arm_matrix_instance_f32 *x_next) {
    (void)x;
    (void)u;
    (void)x_next;
}

static void rotation_df(const arm_matrix_instance_f32 *x, const arm_matrix_instance_f32 *u, arm_matrix_instance_f32 *x_next) {
    (void)x;
    (void)u;
    (void)x_next;
}

static float Q_data[7*7] = {
    1, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 1
};

ekf_system_model_t rotation_model = {
    .Q.numRows = 7,
    .Q.numCols = 7,
    .Q.pData = Q_data,
    .f = rotation_f,
    .df = rotation_df
};

static void accelerometer_h(const arm_matrix_instance_f32 *x, arm_matrix_instance_f32 *z) {
    (void)x;
    (void)z;
}

static void accelerometer_dh(const arm_matrix_instance_f32 *x, arm_matrix_instance_f32 *z) {
    (void)x;
    (void)z;
}

static float acc_R_data[3*3] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};

ekf_measurement_model_t accelerometer_model = {
    .R.numRows = 3,
    .R.numCols = 3,
    .R.pData = acc_R_data,
    .h = accelerometer_h,
    .dh = accelerometer_dh
};

static void magnetometer_h(const arm_matrix_instance_f32 *x, arm_matrix_instance_f32 *z) {
    (void)x;
    (void)z;
}

static void magnetometer_dh(const arm_matrix_instance_f32 *x, arm_matrix_instance_f32 *z) {
    (void)x;
    (void)z;
}

static float mag_R_data[3*3] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};

ekf_measurement_model_t magnetometer_model = {
    .R.numRows = 3,
    .R.numCols = 3,
    .R.pData = mag_R_data,
    .h = magnetometer_h,
    .dh = magnetometer_dh
};

EKF_PREDICT(7, 3)
EKF_CORRECT(7, 3)
