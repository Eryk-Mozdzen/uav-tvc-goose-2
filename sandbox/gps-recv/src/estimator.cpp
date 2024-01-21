#include "estimator.h"
#include "ekf.h"

using math::Matrix;
using math::Vector;

class DynamicsModel : public math::EKF<6>::SystemModel {
    static constexpr float t = 1.f/ESTIMATOR_PREDICTION_FREQUENCY;

    const Matrix<6, 6> A {
        {1, 0, t, 0, t*t/2, 0},
        {0, 1, 0, t, 0, t*t/2},
        {0, 0, 1, 0, t, 0},
        {0, 0, 0, 1, 0, t},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1}
    };

public:
    Vector<6> f(const Vector<6> &x) const {
        return A*x;
    }

	Matrix<6, 6> df(const Vector<6> &x) const {
        (void)x;
        return A;
    }
};

class AccelerometerModel : public math::EKF<6>::MeasurementModel<2> {
    const Matrix<2, 6> A {
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1}
    };

public:
    AccelerometerModel() : MeasurementModel{Matrix<2, 2>::Identity()} {

    }

    Vector<2> h(const Vector<6> &x) const {
        return A*x;
    }

    Matrix<2, 6> dh(const Vector<6> &x) const {
        (void)x;
        return A;
    }
};

class GPSModel : public math::EKF<6>::MeasurementModel<2> {
    const Matrix<2, 6> A {
        {1, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0}
    };

public:
    GPSModel() : MeasurementModel{Matrix<2, 2>::Identity()} {

    }

    Vector<2> h(const Vector<6> &x) const {
        return A*x;
    }

    Matrix<2, 6> dh(const Vector<6> &x) const {
        (void)x;
        return A;
    }
};

static math::EKF<6> ekf(Eigen::Vector<float, 6>::Zero());
static DynamicsModel dynamics;
static AccelerometerModel accelerometer;
static GPSModel gps;

static Eigen::Vector3f GPStoENU(const float lat, const float lon, const float lat_ref, const float lon_ref) {
    constexpr float r = 6371000.f;

    const Eigen::Vector3f ecef {
        r*std::cos(lat)*std::cos(lon),
        r*std::cos(lat)*std::sin(lon),
        r*std::sin(lat)
    };

    const Eigen::Vector3f ecef_ref {
        r*std::cos(lat_ref)*std::cos(lon_ref),
        r*std::cos(lat_ref)*std::sin(lon_ref),
        r*std::sin(lat_ref)
    };

    const float s_phi = std::sin(lat_ref);
    const float c_phi = std::cos(lat_ref);
    const float s_lambda = std::sin(lon_ref);
    const float c_lambda = std::cos(lon_ref);

    const Eigen::Matrix3f R {
        {-s_lambda, c_lambda, 0},
        {-s_phi*c_lambda, -s_phi*s_lambda, c_phi},
        {c_phi*c_lambda, c_phi*s_lambda, s_phi}
    };

    const Eigen::Vector3f enu = R*(ecef - ecef_ref);

    return enu;
}

extern "C"
void Estimator_FeedAccelerometer(const float x, const float y) {
    ekf.correct(accelerometer, {x, y});
}

extern "C"
void Estimator_FeedGPS(const float latitude, const float longitude) {
    constexpr float refLatitude = 0.950871f;  // 54*28'51.2''
    constexpr float refLongitude = 0.323817f; // 18*33'12.1''

    const Eigen::Vector3f pos = GPStoENU(latitude, longitude, refLatitude, refLongitude);
    const float x = pos(0);
    const float y = pos(1);

    ekf.correct(gps, {x, y});
}

extern "C"
void Estimator_Predict() {
    ekf.predict(dynamics);
}

extern "C"
void Estimator_GetState(Estimator_State_t *state) {
    const Eigen::Vector<float, 6> x = ekf.getState();

    state->position[0] = x(0);
    state->position[1] = x(1);

    state->velocity[0] = x(2);
    state->velocity[1] = x(3);
}
