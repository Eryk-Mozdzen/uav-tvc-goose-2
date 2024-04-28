#include <math.h>

#include "ekf.h"
#include "estimator.h"

#define GYROSCOPE_VARIANCE      1
#define ACCELEROMETER_VARIANCE  100
#define MAGNETOMETER_VARIANCE   100

static float x_data[5] = {
    1, 0, 0, 0, // qw, qx, qy, qz
    0           // magnetic dip angle
};

static float P_data[5*5] = {
    1, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 1
};

ekf_t ekf = {
    .x.numRows = 5,
    .x.numCols = 1,
    .x.pData = x_data,
    .P.numRows = 5,
    .P.numCols = 5,
    .P.pData = P_data
};

static void rotation_f(const arm_matrix_instance_f32 *x, const arm_matrix_instance_f32 *u, arm_matrix_instance_f32 *x_next) {
    const float qw = x->pData[0];
    const float qx = x->pData[1];
    const float qy = x->pData[2];
    const float qz = x->pData[3];

    const float wx = u->pData[0];
    const float wy = u->pData[1];
    const float wz = u->pData[2];

    const float dt2 = ESTIMATOR_PREDICT_DT/2.f;

    x_next->pData[0] = qw - dt2*wx*qx - dt2*wy*qy - dt2*wz*qz;
    x_next->pData[1] = qx + dt2*wx*qw - dt2*wy*qz + dt2*wz*qy;
    x_next->pData[2] = qy + dt2*wx*qz + dt2*wy*qw - dt2*wz*qx;
    x_next->pData[3] = qz - dt2*wx*qy + dt2*wy*qx + dt2*wz*qw;
    x_next->pData[4] = x->pData[4];
}

static void rotation_df(const arm_matrix_instance_f32 *x, const arm_matrix_instance_f32 *u, arm_matrix_instance_f32 *x_next) {
    (void)x;

    const float wx = u->pData[0];
    const float wy = u->pData[1];
    const float wz = u->pData[2];

    const float dt2 = ESTIMATOR_PREDICT_DT/2.f;

    x_next->pData[0*5 + 0] = 1.f;
    x_next->pData[0*5 + 1] = -dt2*wx;
    x_next->pData[0*5 + 2] = -dt2*wy;
    x_next->pData[0*5 + 3] = -dt2*wz;
    x_next->pData[0*5 + 4] = 0.f;

    x_next->pData[1*5 + 0] = dt2*wx;
    x_next->pData[1*5 + 1] = 1.f;
    x_next->pData[1*5 + 2] = dt2*wz;
    x_next->pData[1*5 + 3] = -dt2*wy;
    x_next->pData[1*5 + 4] = 0.f;

    x_next->pData[2*5 + 0] = dt2*wy;
    x_next->pData[2*5 + 1] = -dt2*wz;
    x_next->pData[2*5 + 2] = 1.f;
    x_next->pData[2*5 + 3] = dt2*wx;
    x_next->pData[2*5 + 4] = 0.f;

    x_next->pData[3*5 + 0] = dt2*wz;
    x_next->pData[3*5 + 1] = dt2*wy;
    x_next->pData[3*5 + 2] = -dt2*wx;
    x_next->pData[3*5 + 3] = 1.f;
    x_next->pData[3*5 + 4] = 0.f;

    x_next->pData[4*5 + 0] = 0.f;
    x_next->pData[4*5 + 1] = 0.f;
    x_next->pData[4*5 + 2] = 0.f;
    x_next->pData[4*5 + 3] = 0.f;
    x_next->pData[4*5 + 4] = 1.f;
}

static float Q_data[5*5] = {
    GYROSCOPE_VARIANCE, 0, 0, 0, 0,
    0, GYROSCOPE_VARIANCE, 0, 0, 0,
    0, 0, GYROSCOPE_VARIANCE, 0, 0,
    0, 0, 0, GYROSCOPE_VARIANCE, 0,
    0, 0, 0, 0, GYROSCOPE_VARIANCE
};

ekf_system_model_t rotation_model = {
    .Q.numRows = 5,
    .Q.numCols = 5,
    .Q.pData = Q_data,
    .f = rotation_f,
    .df = rotation_df
};

static void accelerometer_h(const arm_matrix_instance_f32 *x, arm_matrix_instance_f32 *z) {
    const float qw = x->pData[0];
    const float qx = x->pData[1];
    const float qy = x->pData[2];
    const float qz = x->pData[3];

    z->pData[0] = -(2.f*qx*qz - 2.f*qy*qw);
    z->pData[1] = -(2.f*qy*qz + 2.f*qx*qw);
    z->pData[2] = -(1.f - 2.f*qx*qx - 2.f*qy*qy);
}

static void accelerometer_dh(const arm_matrix_instance_f32 *x, arm_matrix_instance_f32 *z) {
    const float qw = x->pData[0];
    const float qx = x->pData[1];
    const float qy = x->pData[2];
    const float qz = x->pData[3];

    z->pData[0*5 + 0] =  2.f*qy;
    z->pData[0*5 + 1] = -2.f*qz;
    z->pData[0*5 + 2] =  2.f*qw;
    z->pData[0*5 + 3] = -2.f*qx;
    z->pData[0*5 + 4] = 0.f;

    z->pData[1*5 + 0] = -2.f*qx;
    z->pData[1*5 + 1] = -2.f*qw;
    z->pData[1*5 + 2] = -2.f*qz;
    z->pData[1*5 + 3] = -2.f*qy;
    z->pData[1*5 + 4] = 0.f;

    z->pData[2*5 + 0] = 0.f;
    z->pData[2*5 + 1] = 4.f*qx;
    z->pData[2*5 + 2] = 4.f*qy;
    z->pData[2*5 + 3] = 0.f;
    z->pData[2*5 + 4] = 0.f;
}

static float acc_R_data[3*3] = {
    ACCELEROMETER_VARIANCE, 0, 0,
    0, ACCELEROMETER_VARIANCE, 0,
    0, 0, ACCELEROMETER_VARIANCE
};

ekf_measurement_model_t accelerometer_model = {
    .R.numRows = 3,
    .R.numCols = 3,
    .R.pData = acc_R_data,
    .h = accelerometer_h,
    .dh = accelerometer_dh
};

static void magnetometer_h(const arm_matrix_instance_f32 *x, arm_matrix_instance_f32 *z) {
    const float qw = x->pData[0];
    const float qx = x->pData[1];
    const float qy = x->pData[2];
    const float qz = x->pData[3];
    const float c = cosf(x->pData[4]);
    const float s = sinf(x->pData[4]);

    z->pData[0] = c*(2.f*qx*qy + 2.f*qz*qw) + s*(2.f*qx*qz - 2.f*qy*qw);
    z->pData[1] = c*(1.f - 2.f*qx*qx - 2.f*qz*qz) + s*(2.f*qy*qz + 2.f*qx*qw);
    z->pData[2] = c*(2.f*qy*qz - 2.f*qx*qw) + s*(1.f - 2.f*qx*qx - 2.f*qy*qy);
}

static void magnetometer_dh(const arm_matrix_instance_f32 *x, arm_matrix_instance_f32 *z) {
    const float qw = x->pData[0];
    const float qx = x->pData[1];
    const float qy = x->pData[2];
    const float qz = x->pData[3];
    const float c = cosf(x->pData[4]);
    const float s = sinf(x->pData[4]);

    z->pData[0*5 + 0] = 2.f*qz*c - 2.f*qy*s;
    z->pData[0*5 + 1] = 2.f*qy*c + 2.f*qz*s;
    z->pData[0*5 + 2] = 2.f*qx*c - 2.f*qw*s;
    z->pData[0*5 + 3] = 2.f*qw*c + 2.f*qy*s;
    z->pData[0*5 + 4] = -s*(2.f*qx*qy + 2.f*qz*qw) + c*(2.f*qx*qz - 2.f*qy*qw);

    z->pData[1*5 + 0] = 2.f*qx*s;
    z->pData[1*5 + 1] = -4.f*qx*c + 2.f*qw*s;
    z->pData[1*5 + 2] = 2.f*qz*s;
    z->pData[1*5 + 3] = -4.f*qz*c + 2.f*qy*s;
    z->pData[1*5 + 4] = -s*(1.f - 2.f*qx*qx - 2.f*qz*qz) + c*(2.f*qy*qz + 2.f*qx*qw);

    z->pData[2*5 + 0] = -2.f*qx*c;
    z->pData[2*5 + 1] = -2.f*qw*c - 4.f*qx*s;
    z->pData[2*5 + 2] = 2.f*qz*c - 4.f*qy*s;
    z->pData[2*5 + 3] = 2.f*qy*c;
    z->pData[2*5 + 4] = -s*(2.f*qy*qz - 2.f*qx*qw) + c*(1.f - 2.f*qx*qx - 2.f*qy*qy);
}

static float mag_R_data[3*3] = {
    MAGNETOMETER_VARIANCE, 0, 0,
    0, MAGNETOMETER_VARIANCE, 0,
    0, 0, MAGNETOMETER_VARIANCE
};

ekf_measurement_model_t magnetometer_model = {
    .R.numRows = 3,
    .R.numCols = 3,
    .R.pData = mag_R_data,
    .h = magnetometer_h,
    .dh = magnetometer_dh
};

EKF_PREDICT(5, 3)
EKF_CORRECT(5, 3)
