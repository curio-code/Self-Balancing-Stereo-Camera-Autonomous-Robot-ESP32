#include "Kalman.h"

Kalman::Kalman() {

    // set default values for the process and the measurement variances
    V1_angle   = 0.001f;      // defaults: 0.001 - try: 0.001
    V1_bias    = 0.001f;      // defautls: 0.003 - try: 0.003
    V2_measure = 3.003f;      // defaults: 0.030 - try: 9.900

    // initialization of angle and bias
    angle = 0.0f;
    bias = 0.0f;

    // initialization of the filter gain
    e = 0.0f;

    // initialization of the error covariance matrix
    // the values of the matrix are all set to 0 because
    // we know the initial position of the robot and
    // the sensor offsets (computed through the calibration phase)
    P[0][0] = 0.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

// accAngle in degrees, gyroRate in degrees/second, deltaTime in seconds
float Kalman::getAngle(float accAngle, float gyroRate, float deltaTime) {

    // compute the a priori estimate of the angle of the state vector x = [angle bias]'
    // (' stands for transposed)
    rate = gyroRate - bias;
    angle += deltaTime * rate;

    // compute the a priori estimate of the error covariance matrix P
    // P(t|t-1) = F(t)P(t-1|t-1)F(t)' + V1*Δt
    P[0][0] += deltaTime * (deltaTime*P[1][1] - P[0][1] - P[1][0] + V1_angle);
    P[0][1] -= deltaTime * P[1][1];
    P[1][0] -= deltaTime * P[1][1];
    P[1][1] += V1_bias * deltaTime;

    // compute the measurement prediction covariance
    // S(t) = H(t)P(t|t-1)H(t)' + V2
    float S = P[0][0] + V2_measure;


    // compute the filter gain
    // K(t) = P(t|t-1)H(t)'S(t)^-1
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // calculate the innovation (measurement residual)
    // e(t) = y(t) - H(t)x(t|t-1)
    e = accAngle - angle;

    // state estimate update
    // x(t|t) = x(t|t-1) + K(t)e(t)
    angle += K[0] * e;
    bias += K[1] * e;

    // state covariance update
    // P(t|t) = P(t|t-1) - K(t)S(t)K(t)'
    //        = (I - K(t)H(t))P(t|t-1)
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};

// set the starting angle of the robot with respect to the vertical position
void Kalman::setAngle(float angle) {
    this->angle = angle;
};

float Kalman::getBias() {
  return this->bias;
};

float Kalman::getInnovation() {
  return this->e;
};
