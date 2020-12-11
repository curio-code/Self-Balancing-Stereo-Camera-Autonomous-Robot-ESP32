#ifndef _Kalman_h_
#define _Kalman_h_

class Kalman {

public:
    // Default constructor
    Kalman();

    /**
     * Implements the kalman filter in order to filter the angle of the robot
     * with respect to the measurement from the MPU6050
     *
     * - Parameters:
     *      - accAngle  : the accelerometer measurement (represents the output of the system) [in degrees]
     *      - gyroRate  : the gyroscope measurement (represents the deterministic input of the system) [in degrees/sec]
     *      - deltaTime : the time interval since the last update of the system [in sec]
     * - Returns: the estimate measurement of the angle of the robot with respect to the vertical position.
     */
    float getAngle(float accAngle, float gyroRate, float deltaTime);

    /**
     * Set the starting angle of the robot with respect to the vertical position
     *
     * - Parameters:
     *      - angle : the value of the starting angle to be set [in degrees]
     */
    void setAngle(float angle);

    float getBias();
    float getInnovation();

public:
    /** Kalman filter variables
     * V1_angle      : the process noise variance specific to the accelerometer
     * V1_bias       : the process noise variance specific to the gyroscope
     * V2_measure    : the measurement noise variance of the angle computed from the accelerometer
     * angle         : the updated angle computed through the Kalman filter
     * bias          : the updated gyroscope bias computed through the Kalman filter
     * rate          : the unbiased rate
     * P             : the error covariance matrix [2X2]
     * e             : kalman innovation term computed in the previous run of getAngle function
     */
    float V1_angle;
    float V1_bias;
    float V2_measure;
    float angle;
    float bias;
    float rate;
    float P[2][2];
    float e;

};

#endif
