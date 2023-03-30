#include "KalmanFilter.h"

void KalmanFilter::invert4x4(float * src, float * dst)
{
    float det;

    /* Compute adjoint: */
    dst[0] = +src[5] * src[10] * src[15] - src[5] * src[11] * src[14]
            - src[9] * src[6] * src[15] + src[9] * src[7] * src[14]
            + src[13] * src[6] * src[11] - src[13] * src[7] * src[10];

    dst[1] = -src[1] * src[10] * src[15] + src[1] * src[11] * src[14]
            + src[9] * src[2] * src[15] - src[9] * src[3] * src[14]
            - src[13] * src[2] * src[11] + src[13] * src[3] * src[10];

    dst[2] = +src[1] * src[6] * src[15] - src[1] * src[7] * src[14]
            - src[5] * src[2] * src[15] + src[5] * src[3] * src[14]
            + src[13] * src[2] * src[7] - src[13] * src[3] * src[6];

    dst[3] = -src[1] * src[6] * src[11] + src[1] * src[7] * src[10]
            + src[5] * src[2] * src[11] - src[5] * src[3] * src[10]
            - src[9] * src[2] * src[7] + src[9] * src[3] * src[6];

    dst[4] = -src[4] * src[10] * src[15] + src[4] * src[11] * src[14]
            + src[8] * src[6] * src[15] - src[8] * src[7] * src[14]
            - src[12] * src[6] * src[11] + src[12] * src[7] * src[10];

    dst[5] = +src[0] * src[10] * src[15] - src[0] * src[11] * src[14]
            - src[8] * src[2] * src[15] + src[8] * src[3] * src[14]
            + src[12] * src[2] * src[11] - src[12] * src[3] * src[10];

    dst[6] = -src[0] * src[6] * src[15] + src[0] * src[7] * src[14]
            + src[4] * src[2] * src[15] - src[4] * src[3] * src[14]
            - src[12] * src[2] * src[7] + src[12] * src[3] * src[6];

    dst[7] = +src[0] * src[6] * src[11] - src[0] * src[7] * src[10]
            - src[4] * src[2] * src[11] + src[4] * src[3] * src[10]
            + src[8] * src[2] * src[7] - src[8] * src[3] * src[6];

    dst[8] = +src[4] * src[9] * src[15] - src[4] * src[11] * src[13]
            - src[8] * src[5] * src[15] + src[8] * src[7] * src[13]
            + src[12] * src[5] * src[11] - src[12] * src[7] * src[9];

    dst[9] = -src[0] * src[9] * src[15] + src[0] * src[11] * src[13]
            + src[8] * src[1] * src[15] - src[8] * src[3] * src[13]
            - src[12] * src[1] * src[11] + src[12] * src[3] * src[9];

    dst[10] = +src[0] * src[5] * src[15] - src[0] * src[7] * src[13]
            - src[4] * src[1] * src[15] + src[4] * src[3] * src[13]
            + src[12] * src[1] * src[7] - src[12] * src[3] * src[5];

    dst[11] = -src[0] * src[5] * src[11] + src[0] * src[7] * src[9]
            + src[4] * src[1] * src[11] - src[4] * src[3] * src[9]
            - src[8] * src[1] * src[7] + src[8] * src[3] * src[5];

    dst[12] = -src[4] * src[9] * src[14] + src[4] * src[10] * src[13]
            + src[8] * src[5] * src[14] - src[8] * src[6] * src[13]
            - src[12] * src[5] * src[10] + src[12] * src[6] * src[9];

    dst[13] = +src[0] * src[9] * src[14] - src[0] * src[10] * src[13]
            - src[8] * src[1] * src[14] + src[8] * src[2] * src[13]
            + src[12] * src[1] * src[10] - src[12] * src[2] * src[9];

    dst[14] = -src[0] * src[5] * src[14] + src[0] * src[6] * src[13]
            + src[4] * src[1] * src[14] - src[4] * src[2] * src[13]
            - src[12] * src[1] * src[6] + src[12] * src[2] * src[5];

    dst[15] = +src[0] * src[5] * src[10] - src[0] * src[6] * src[9]
            - src[4] * src[1] * src[10] + src[4] * src[2] * src[9]
            + src[8] * src[1] * src[6] - src[8] * src[2] * src[5];

    /* Compute determinant: */

    det = +src[0] * dst[0] + src[1] * dst[4] + src[2] * dst[8]
            + src[3] * dst[12];

    /* Multiply adjoint with reciprocal of determinant: */
    if(det == 0.0){
    	det = 0.0;
    }else{
    	det = 1.0f / det;
    }

    dst[0] *= det;
    dst[1] *= det;
    dst[2] *= det;
    dst[3] *= det;
    dst[4] *= det;
    dst[5] *= det;
    dst[6] *= det;
    dst[7] *= det;
    dst[8] *= det;
    dst[9] *= det;
    dst[10] *= det;
    dst[11] *= det;
    dst[12] *= det;
    dst[13] *= det;
    dst[14] *= det;
    dst[15] *= det;
}

float *KalmanFilter::predict()
{
    float *_P = ErrorCovPost;
    float *_Q = ProcessNoiseCov;
    float *_A = TransitionMatrix;
    float *_X = statePost;

    int i,j,k;
    float sum;
    //X_new = A*X
    for (i = 0; i < KALMAN_STATE_SIZE; i++)
    {
        sum = 0;
        for (k = 0; k < KALMAN_STATE_SIZE; k++)
        {
            sum += (_A[k + (i*KALMAN_STATE_SIZE)] * _X[k]);
        }
        Xnew[i] = sum;
    }

    float _Ptemp1[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE];
    //P_new = A*P*A' + Q
    //A*P
    for (i = 0; i < KALMAN_STATE_SIZE; i++)
    {
        for (j = 0; j < KALMAN_STATE_SIZE; j++)
        {
            sum = 0;
            for (k = 0; k < KALMAN_STATE_SIZE; k++)
            {
                sum += (_A[k + (i * KALMAN_STATE_SIZE)] * _P[j + (k * KALMAN_STATE_SIZE)]);
            }
            _Ptemp1[j + (i * KALMAN_STATE_SIZE)] = sum;
        }
    }
    float _Ptemp2[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE];
    //A*P*A'
    for (i = 0; i < KALMAN_STATE_SIZE; i++)
    {
        for (j = 0; j < KALMAN_STATE_SIZE; j++)
        {
            sum = 0;
            for (k = 0; k < KALMAN_STATE_SIZE; k++)
            {
                sum += (_Ptemp1[k + (i*KALMAN_STATE_SIZE)] * _A[k + (j*KALMAN_STATE_SIZE)]);
            }
            
            _Ptemp2[j + (i*KALMAN_STATE_SIZE)] = sum;
        }
    }

    //P_new = A*P*A' + Q

    for (i = 0; i < KALMAN_STATE_SIZE*KALMAN_STATE_SIZE; i++)
    {
        Pnew[i] = _Ptemp2[i] + _Q[i];
    }

    for (i = 0; i < KALMAN_STATE_SIZE*KALMAN_STATE_SIZE; i++)
    {
        ErrorCovPost[i] = Pnew[i];
    }

    return Xnew;
}

void setIdentity(float *arr, const int cols, const int rows, const float num)
{
    int i,j;
    memset(arr, 0, cols*rows*sizeof(float));

    for (j = 0; j < rows; j++)
    {
        for ( i = 0; i < cols; i++)
        {
            if( i == j)
            {
                arr[i + j*cols] = num;
            }
        }
    }
}

void setIdentity(float *arr, const int cols, const int rows)
{
    int i,j;
    memset(arr, 0, cols*rows*sizeof(float));

    for (j = 0; j < rows; j++)
    {
        for ( i = 0; i < cols; i++)
        {
            if (i == j)
            {
                arr[i + j * cols] = 1;
            }
        }
    }
}

void KalmanFilter::correct(const float *measure)
{
    int i, j, k, r1, c1, r2, c2;
    float sum;
    int numEl;
    
    float residual[KALMAN_MEASURED_SIZE];

    float *_P = ErrorCovPost;
    //float *_Q = ProcessNoiseCov;
    //float *_A = TransitionMatrix;
    float *_X = statePost;
    float *_H = MeasurementMatrix;
    float *_R = MeasurementNoiseCov;

    float *_Ptemp1 = m_Temp1;
    float *_Ptemp2 = m_Temp2;
    float *_Ptemp3 = m_Temp3;

    r1 = KALMAN_STATE_SIZE;
    c1 = r1;
    r2 = KALMAN_MEASURED_SIZE;
    c2 = KALMAN_STATE_SIZE;

    /* kalman gain*/
    /*K = P1*H'; [7x7 * 7x4] = 7x4*/
    for (i = 0; i < r1; i++)
    {
        for (j = 0; j < r2; j++)
        {
            sum = (float) 0.0;

            for (k = 0; k < c1; k++)
            {
                sum += (Pnew[k + (i * c1)] * _H[k + (j * c2)]);
            }

            _Ptemp1[j + (i * r2)] = sum;
        }
    }

    r1 = KALMAN_MEASURED_SIZE;
    c1 = KALMAN_STATE_SIZE;
    r2 = KALMAN_STATE_SIZE;
    c2 = KALMAN_MEASURED_SIZE;
    for (i = 0; i < r1; i++)
    {
        for (j = 0; j < c2; j++)
        {
            sum = (float) 0.0;

            for (k = 0; k < c1; k++)
            {
                sum += (_H[k + (i * c1)] * _Ptemp1[j + (k * c2)]);
            }

            _Ptemp2[j + (i * c2)] = sum;
        }
    }

    /*pTemp2 = (H*P1*H') + R*/
    numEl = r1 * c2; // KALMAN_MEASURED_SIZE*KALMAN_MEASURED_SIZE

    for (i = 0; i < numEl; i++)
    {
        _Ptemp2[i] = _Ptemp2[i] + _R[i];
    }

    invert4x4(_Ptemp2, _Ptemp3);

    /* K = K* inv((H*P1*H') + R)*/
    r1 = KALMAN_STATE_SIZE;
    c1 = KALMAN_MEASURED_SIZE;
    r2 = KALMAN_MEASURED_SIZE;
    c2 = KALMAN_MEASURED_SIZE;
    for (i = 0; i < r1; i++)
    {
        for (j = 0; j < c2; j++)
        {
            sum = (float) 0.0;

            for (k = 0; k < c1; k++)
            {
                sum += (_Ptemp1[k + (i * c1)] * _Ptemp3[j + (k * c2)]);
            }

            KalmanGainMatrix[j + (i * c2)] = sum;
        }
    }

    /* pTemp1 = H*X1*/
    r1 = KALMAN_MEASURED_SIZE;
    c1 = KALMAN_STATE_SIZE;
    r2 = KALMAN_STATE_SIZE;
    c2 = 1;
    for (i = 0; i < r1; i++)
    {
        sum = (float) 0.0;

        for (k = 0; k < c1; k++)
        {
            sum += (_H[k + (i * c1)] * Xnew[k]);
        }
        _Ptemp1[i] = sum;
    }

    /* Residual = Z - H*X1*/
    for (i = 0; i < KALMAN_MEASURED_SIZE; i++)
    {
        residual[i] = measure[i] - _Ptemp1[i];
    }

    /* K*Residual*/
    r1 = KALMAN_STATE_SIZE;
    c1 = KALMAN_MEASURED_SIZE;
    r2 = KALMAN_MEASURED_SIZE;
    c2 = 1;
    for (i = 0; i < r1; i++)
    {
        sum = (float) 0.0;

        for (k = 0; k < c1; k++)
        {
            sum += (KalmanGainMatrix[k + (i * c1)] * residual[k]);
        }

        _Ptemp2[i] = (sum);
    }
    
    for (i = 0; i < r1; i++)
    {
        _X[i]         = Xnew[i] + _Ptemp2[i];
        statePost[i] = Xnew[i] + _Ptemp2[i];
    }

    /* update error covariance*/
    /* K*H*/
    r1 = KALMAN_STATE_SIZE;
    c1 = KALMAN_MEASURED_SIZE;
    r2 = KALMAN_MEASURED_SIZE;
    c2 = KALMAN_STATE_SIZE;
    for (i = 0; i < r1; i++)
    {
        for (j = 0; j < c2; j++)
        {
            sum = (float) 0.0;

            for (k = 0; k < c1; k++)
            {
                sum += (KalmanGainMatrix[k + (i * c1)] * _H[j + (k * c2)]);
            }

            _Ptemp2[j + (i * c2)] = sum;
        }
    }


    /* (K*H)*Pnew*/
    r1 = KALMAN_STATE_SIZE;
    c1 = KALMAN_STATE_SIZE;
    r2 = KALMAN_STATE_SIZE;
    c2 = KALMAN_STATE_SIZE;

    for (i = 0; i < r1; i++)
    {
        for (j = 0; j < c2; j++)
        {
            sum = (float)0.0;

            for (k = 0; k < c1; k++)
            {
                sum += (_Ptemp2[k + (i * c1)] * Pnew[j + (k * c2)]);
            }

            _Ptemp3[j + (i * c2)] = sum;
        }
    }

    /* Pnew - ((K*H)*Pnew)*/
    for (i = 0; i < (r1 * c2); i++)
    {
        ErrorCovPost[i] = Pnew[i] - _Ptemp3[i];
        _P[i]           = Pnew[i] - _Ptemp3[i];
    }

}