#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#define KALMAN_STATE_SIZE 7
#define KALMAN_MEASURED_SIZE 4

#include <iostream>
#include <stdio.h>
#include <string.h>
class KalmanFilter
{
public:
    KalmanFilter(){}
    
    KalmanFilter(int dynamParams, int measureParams, int controlParams){}

    ~KalmanFilter(){}
    
    //!< Matrix X (7x1)
    float statePost[KALMAN_STATE_SIZE];
    //!< Matrix A (7x7)
    float TransitionMatrix[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE]; 
    //!< Matrix H (7x4)
    float MeasurementMatrix[KALMAN_STATE_SIZE*KALMAN_MEASURED_SIZE];
    //!< Matrix Q (7x7)
    float ProcessNoiseCov[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE];
    //!< Matrix R (4x4)
    float MeasurementNoiseCov[KALMAN_MEASURED_SIZE*KALMAN_MEASURED_SIZE];
    //!< Matrix P (7x7)
    float ErrorCovPost[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE]; //7x7 matrix
    //!< Matrix Xnew (7x1) -> predicted X
    float Xnew[KALMAN_STATE_SIZE];
    //!< Matrix Xnew (7x1) -> predicted P
    float Pnew[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE];
    
    float *predict();
    void correct(const float *measure);

private:
    //4x4 inverse matrix
    void invert4x4(float * src, float * dst);
    
    //!< Matrix K (4x7)
    float KalmanGainMatrix[KALMAN_MEASURED_SIZE*KALMAN_STATE_SIZE]; // 4x7 matrix

    float m_Temp1[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE];
    float m_Temp2[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE];
    float m_Temp3[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE];
};

void setIdentity(float *arr, const int cols, const int rows);
void setIdentity(float *arr, const int cols, const int rows, const float num);
#endif