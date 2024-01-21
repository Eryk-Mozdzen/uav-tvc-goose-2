#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#define ESTIMATOR_PREDICTION_FREQUENCY  100.f

typedef struct {
    float position[2];
    float velocity[2];
} Estimator_State_t;

void Estimator_FeedAccelerometer(const float x, const float y);
void Estimator_FeedGPS(const float latitude, const float longitude);

void Estimator_Predict();
void Estimator_GetState(Estimator_State_t *state);

#ifdef __cplusplus
}
#endif

#endif
