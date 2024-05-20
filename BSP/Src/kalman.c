#include "../Inc/kalman.h"

#include <math.h>

//定义一个结构体，用于存储卡尔曼滤波器的参数
typedef struct
{
    float Q_angle; //角度的过程噪声
    float Q_bias;  //角速度的过程噪声
    float R_measure; //测量噪声
    float angle; //角度
    float bias; //角速度
    float rate; //角速度
    
    float F[2][2]; //状态转移矩阵
    float B[2][1]; //控制输入矩阵
    float H[2]; //观测矩阵

    float P[2][2]; //协方差矩阵
    float K[2]; //卡尔曼增益，是2维的
    

}Kalman;

float Matrix_plus_2_2(float A[2][2], float B[2][2])
{
    float C[2][2];
    C[0][0] = A[0][0] + B[0][0];
    C[0][1] = A[0][1] + B[0][1];
    C[1][0] = A[1][0] + B[1][0];
    C[1][1] = A[1][1] + B[1][1];

    return C;
}

float Matrix_minus_2_2(float A[2][2], float B[2][2])
{
    float C[2][2];
    C[0][0] = A[0][0] - B[0][0];
    C[0][1] = A[0][1] - B[0][1];
    C[1][0] = A[1][0] - B[1][0];
    C[1][1] = A[1][1] - B[1][1];

    return C;
}

float Matrix_multiply_2_2(float A[2][2], float B[2][2])
{
    float C[2][2];
    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    C[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1];
    C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
    C[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1];

    return C;
}

float Matrix_inverse_2_2(float A[2][2])
{
    float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    float C[2][2];
    C[0][0] = A[1][1] / det;
    C[0][1] = -A[0][1] / det;
    C[1][0] = -A[1][0] / det;
    C[1][1] = A[0][0] / det;

    return C;
}

void kalman_init(Kalman *kalman)
{
    kalman->Q_angle = 0.001;
    kalman->Q_bias = 0.003;
    kalman->R_measure = 0.03;
    kalman->angle = 0.0;
    kalman->bias = 0.0;
    kalman->rate = 0.0;
    kalman->P[0][0] = 1.0;
    kalman->P[0][1] = 0.0;
    kalman->P[1][0] = 0.0;
    kalman->P[1][1] = 1.0;
}

void kalman_update(Kalman *kalman, float newAngle, float newRate, float dt)
{
    kalman->rate = newRate - kalman->bias;
    kalman->angle += dt * kalman->rate;