#include "../Inc/kalman.h"

#include <math.h>

//定义一个结构体，用于存储卡尔曼滤波器的参数
typedef struct
{
    float Q[2][2]; //角度的过程噪声
    float R[2][2]; //测量噪声
    
    float F[2][2]; //状态转移矩阵
    float B[2][2]; //控制输入矩阵
    float H[2]; //观测矩阵

    float P[2][2]; //协方差矩阵
    float K[2]; //卡尔曼增益，是2维的

    

}Kalman;

typedef struct
{
    float roll;
    float pitch;
    float yaw;

}angle;

angle angle_from_acc;
angle angle_from_gyro;

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

float Matrix_multiple_2_1(float A[2][2], float B[2][1])
{
    float C[2][1];
    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];

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

float Matrix_transpose_2_2(float A[2][2])
{
    float C[2][2];
    C[0][0] = A[0][0];
    C[0][1] = A[1][0];
    C[1][0] = A[0][1];
    C[1][1] = A[1][1];

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

void kalman_predict(Kalman *kalman, angle angle_from_acc, angle angle_from_gyro, float dt)
{
    float kalman_x_priori;
    float F[2][2];
    F = kalman->F;

}