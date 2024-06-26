#include "../Inc/mpu6050.h"

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f
#define M_PI 3.14159265359
short gyro[3], accel[3], sensors;
//float Pitch,Roll; 
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
// static signed char gyro_orientation[9] = {-1, 0, 0,
//                                            0,-1, 0,
//                                            0, 0, 1};
float myroll = 0;
float mypitch = 0;
float myyaw = 0;
// /* Definitions for defaultTask */
//禁用睡眠模式
void MPU6050_setSleepEnabled(I2C_HandleTypeDef *hi2c, uint8_t enable) {
    uint8_t data[2];
    data[0] = MPU6050_RA_PWR_MGMT_1;  // PWR_MGMT_1寄存器地址
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);
    if(enable) {
        data[1] &= ~0x40; // 清除睡眠位（bit6）
    } else {
        data[1] |= 0x40;  // 设置睡眠位（bit6）
    }
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);
}

//启用/禁用MPU6050作为I2C Master
void MPU6050_setI2CMasterModeEnabled(I2C_HandleTypeDef *hi2c, uint8_t enable) {
    uint8_t data[2];
    data[0] = MPU6050_RA_USER_CTRL; // USER_CTRL寄存器地址
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);
    if(enable) {
        data[1] |= 0x20; // 设置I2C_MASTER_MODE位（bit5）
    } else {
        data[1] &= ~0x20; // 清除I2C_MASTER_MODE位（bit5）
    }
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);
}

//启用/禁用I2C主机模式的旁路
void MPU6050_setI2CBypassEnabled(I2C_HandleTypeDef *hi2c, uint8_t enable) {
    uint8_t data[2];
    data[0] = MPU6050_RA_INT_PIN_CFG; // INT_PIN_CFG寄存器地址
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);
    if(enable) {
        data[1] |= 0x02; // 设置BYPASS_EN位（bit1）
    } else {
        data[1] &= ~0x02; // 清除BYPASS_EN位（bit1）
    }
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);
}

void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2];

    // 唤醒MPU6050并设置时钟源
    data[0] = MPU6050_RA_PWR_MGMT_1;
    data[1] = 0x01;  // 使用X轴陀螺仪作为时钟源
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, data, 2, HAL_MAX_DELAY);

    // 配置加速度计为±2g
    data[0] = MPU6050_RA_ACCEL_CONFIG;
    data[1] = 0x00; //  加速度计±2g
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, data, 2, HAL_MAX_DELAY);

    // 配置陀螺仪为±250度/秒
    data[0] = MPU6050_RA_GYRO_CONFIG;
    data[1] = 0x00; // 陀螺仪±250
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, data, 2, HAL_MAX_DELAY);
    MPU6050_setSleepEnabled(hi2c, 0); // 进入工作状态
    MPU6050_setI2CMasterModeEnabled(hi2c, 0); // 0表示禁用I2C主模式
    MPU6050_setI2CBypassEnabled(hi2c, 1); // 1表示启用Bypass模式


}

void MPU6050_ReadAccelGyro(I2C_HandleTypeDef *hi2c, int16_t* accel, int16_t* gyro) {
    uint8_t data[14];
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 14, HAL_MAX_DELAY);

    // 转换加速度数据
    accel[0] = (data[0] << 8) | data[1];
    accel[1] = (data[2] << 8) | data[3];
    accel[2] = (data[4] << 8) | data[5];
    
    // 转换陀螺仪数据
    gyro[0] = (data[8] << 8) | data[9];
    gyro[1] = (data[10] << 8) | data[11];
    gyro[2] = (data[12] << 8) | data[13];
}

void MPU6050_GetAngle_Acce(I2C_HandleTypeDef *hi2c, float* pitch, float* roll) {
    int16_t accel[3];
    MPU6050_ReadAccelGyro(hi2c, accel, NULL);
    *pitch = atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2])) * 180 / M_PI;
    *roll = atan2(accel[1], sqrt(accel[0] * accel[0] + accel[2] * accel[2])) * 180 / M_PI;
    //*roll = atan2(accel[0], accel[2]) * 180 / M_PI;//这个式子角度容易出问题

}//pitch和roll的单位是度

// void MPU6050_GetAngle_Gyro(I2C_HandleTypeDef *hi2c, float* pitch, float* roll, float* yaw) {
//     int16_t gyro[3];
//     MPU6050_ReadAccelGyro(hi2c, NULL, gyro);

//     *roll  = *roll  + gyro[0] + gyro[1] * sin(*roll) * tan(*pitch) + gyro[2] * cos(*roll) * tan(*pitch);
//     *pitch = *pitch + gyro[1] * cos(*roll) - gyro[2] * sin(*roll);
//     *yaw   = *yaw   + gyro[1] * sin(*roll) / cos(*pitch) + gyro[2] * cos(*roll) / cos(*pitch);

// }

void MPU6050_GetGyro_Gyro(I2C_HandleTypeDef *hi2c,float* pitch, float* roll, float* yaw ,float* gyroX, float* gyroY, float* gyroZ){
    int16_t gyro[3];
    MPU6050_ReadAccelGyro(hi2c, NULL, gyro);
    const float DEG_TO_RAD = M_PI / 180.0;  // 度转弧度系数
    const float RAD_TO_DEG = 180.0 / M_PI;  // 弧度转度系数

    // 从陀螺仪读取的原始数据转换为度
    *gyroX = gyro[0] * RAD_TO_DEG;
    *gyroY = gyro[1] * RAD_TO_DEG;
    *gyroZ = gyro[2] * RAD_TO_DEG;

    // 计算基于大地坐标系的角速度
    *gyroX = *gyroX + *gyroY * sin(*roll * DEG_TO_RAD) * tan(*pitch * DEG_TO_RAD) + *gyroZ * cos(*roll * DEG_TO_RAD) * tan(*pitch * DEG_TO_RAD);
    *gyroY = *gyroY * cos(*roll * DEG_TO_RAD) - *gyroZ * sin(*roll * DEG_TO_RAD);
    *gyroZ = *gyroY * sin(*roll * DEG_TO_RAD) / cos(*pitch * DEG_TO_RAD) + *gyroZ * cos(*roll * DEG_TO_RAD) / cos(*pitch * DEG_TO_RAD);
    
}

void MPU6050_GetAngle_Gyro(I2C_HandleTypeDef *hi2c, float* pitch, float* roll, float* yaw, float dt) {
    int16_t gyro[3];
    MPU6050_ReadAccelGyro(hi2c, NULL, gyro);

    const float DEG_TO_RAD = M_PI / 180.0;  // 度转弧度系数
    const float RAD_TO_DEG = 180.0 / M_PI;  // 弧度转度系数

    // 从陀螺仪读取的原始数据转换为度
    float gyroX = gyro[0] * RAD_TO_DEG;
    float gyroY = gyro[1] * RAD_TO_DEG;
    float gyroZ = gyro[2] * RAD_TO_DEG;

    // 计算新的 roll 值
    *roll += (gyroX + gyroY * sin(*roll * DEG_TO_RAD) * tan(*pitch * DEG_TO_RAD) + gyroZ * cos(*roll * DEG_TO_RAD) * tan(*pitch * DEG_TO_RAD)) * dt;

    // 计算新的 pitch 值
    float cosPitch = cos(*pitch * DEG_TO_RAD);
    if (fabs(cosPitch) > 0.0001) {  // 防止除以零
        *pitch += (gyroY * cos(*roll * DEG_TO_RAD) - gyroZ * sin(*roll * DEG_TO_RAD)) * dt;
        // 限制 pitch 防止奇异性
        if (*pitch > 89.0) *pitch = 89.0;
        if (*pitch < -89.0) *pitch = -89.0;
    }

    // 计算新的 yaw 值
    if (fabs(cosPitch) > 0.0001) {  // 同样防止除以零
        *yaw += (gyroY * sin(*roll * DEG_TO_RAD) / cosPitch + gyroZ * cos(*roll * DEG_TO_RAD) / cosPitch) * dt;
    }

    // 保持角度在 0-360 度范围内
    *roll = fmod(*roll, 360.0);
    *pitch = fmod(*pitch, 360.0);
    *yaw = fmod(*yaw, 360.0);
}



void MPU6050_DMP_Init()
{}

void MPU6050_DMP_ReadAngles(float* pitch, float* roll, float* yaw)
{}





































// static  unsigned short inv_row_2_scale(const signed char *row)
// {
//     unsigned short b;

//     if (row[0] > 0)
//         b = 0;
//     else if (row[0] < 0)
//         b = 4;
//     else if (row[1] > 0)
//         b = 1;
//     else if (row[1] < 0)
//         b = 5;
//     else if (row[2] > 0)
//         b = 2;
//     else if (row[2] < 0)
//         b = 6;
//     else
//         b = 7;      // error
//     return b;
// }


// static  unsigned short inv_orientation_matrix_to_scalar(
//     const signed char *mtx)
// {
//     unsigned short scalar;
//     scalar = inv_row_2_scale(mtx);
//     scalar |= inv_row_2_scale(mtx + 3) << 3;
//     scalar |= inv_row_2_scale(mtx + 6) << 6;


//     return scalar;
// }

// static void run_self_test(void)
// {
//     int result;
//     long gyro[3], accel[3];

//     result = mpu_run_self_test(gyro, accel);
//     if (result == 0x7) {
//         /* Test passed. We can trust the gyro data here, so let's push it down
//          * to the DMP.
//          */
//         float sens;
//         unsigned short accel_sens;
//         mpu_get_gyro_sens(&sens);
//         gyro[0] = (long)(gyro[0] * sens);
//         gyro[1] = (long)(gyro[1] * sens);
//         gyro[2] = (long)(gyro[2] * sens);
//         dmp_set_gyro_bias(gyro);
//         mpu_get_accel_sens(&accel_sens);
//         accel[0] *= accel_sens;
//         accel[1] *= accel_sens;
//         accel[2] *= accel_sens;
//         dmp_set_accel_bias(accel);
// 		printf("setting bias succesfully ......\r\n");
//     }
// }



// uint8_t buffer[14];
// int16_t  MPU6050_FIFO[6][11];
// int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;


// /**************************实现函数********************************************
// *函数原型:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
// *功　　能:	    将新的ADC数据更新到 FIFO数组，进行滤波处理
// *******************************************************************************/

// void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
// {
//     unsigned char i ;
//     int32_t sum=0;
//     for(i=1;i<10;i++){	//FIFO 操作
//     MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
//     MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
//     MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
//     MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
//     MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
//     MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
//     }
//     MPU6050_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
//     MPU6050_FIFO[1][9]=ay;
//     MPU6050_FIFO[2][9]=az;
//     MPU6050_FIFO[3][9]=gx;
//     MPU6050_FIFO[4][9]=gy;
//     MPU6050_FIFO[5][9]=gz;

//     sum=0;
//     for(i=0;i<10;i++){	//求当前数组的合，再取平均值
//     sum+=MPU6050_FIFO[0][i];
//     }
//     MPU6050_FIFO[0][10]=sum/10;

//     sum=0;
//     for(i=0;i<10;i++){
//     sum+=MPU6050_FIFO[1][i];
//     }
//     MPU6050_FIFO[1][10]=sum/10;

//     sum=0;
//     for(i=0;i<10;i++){
//     sum+=MPU6050_FIFO[2][i];
//     }
//     MPU6050_FIFO[2][10]=sum/10;

//     sum=0;
//     for(i=0;i<10;i++){
//     sum+=MPU6050_FIFO[3][i];
//     }
//     MPU6050_FIFO[3][10]=sum/10;

//     sum=0;
//     for(i=0;i<10;i++){
//     sum+=MPU6050_FIFO[4][i];
//     }
//     MPU6050_FIFO[4][10]=sum/10;

//     sum=0;
//     for(i=0;i<10;i++){
//     sum+=MPU6050_FIFO[5][i];
//     }
//     MPU6050_FIFO[5][10]=sum/10;
// }

// /**************************实现函数********************************************
// *函数原型:		void MPU6050_setClockSource(uint8_t source)
// *功　　能:	    设置  MPU6050 的时钟源
//  * CLK_SEL | Clock Source
//  * --------+--------------------------------------
//  * 0       | Internal oscillator
//  * 1       | PLL with X Gyro reference
//  * 2       | PLL with Y Gyro reference
//  * 3       | PLL with Z Gyro reference
//  * 4       | PLL with external 32.768kHz reference
//  * 5       | PLL with external 19.2MHz reference
//  * 6       | Reserved
//  * 7       | Stops the clock and keeps the timing generator in reset
// *******************************************************************************/
// void MPU6050_setClockSource(uint8_t source){
//     IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

// }

// /** Set full-scale gyroscope range.
//  * @param range New full-scale gyroscope range value
//  * @see getFullScaleRange()
//  * @see MPU6050_GYRO_FS_250
//  * @see MPU6050_RA_GYRO_CONFIG
//  * @see MPU6050_GCONFIG_FS_SEL_BIT
//  * @see MPU6050_GCONFIG_FS_SEL_LENGTH
//  */
// void MPU6050_setFullScaleGyroRange(uint8_t range) {
//     IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
// }

// /**************************实现函数********************************************
// *函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
// *功　　能:	    设置  MPU6050 加速度计的最大量程
// *******************************************************************************/
// void MPU6050_setFullScaleAccelRange(uint8_t range) {
//     IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
// }

// /**************************实现函数********************************************
// *函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
// *功　　能:	    设置  MPU6050 是否进入睡眠模式
// 				enabled =1   睡觉
// 			    enabled =0   工作
// *******************************************************************************/
// void MPU6050_setSleepEnabled(uint8_t enabled) {
//     IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
// }

// /**************************实现函数********************************************
// *函数原型:		uint8_t MPU6050_getDeviceID(void)
// *功　　能:	    读取  MPU6050 WHO_AM_I 标识	 将返回 0x68
// *******************************************************************************/
// uint8_t MPU6050_getDeviceID(void) {

//     IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
//     return buffer[0];
// }

// /**************************实现函数********************************************
// *函数原型:		uint8_t MPU6050_testConnection(void)
// *功　　能:	    检测MPU6050 是否已经连接
// *******************************************************************************/
// uint8_t MPU6050_testConnection(void) {
//    if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
//    return 1;
//    	else return 0;
// }

// /**************************实现函数********************************************
// *函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
// *功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
// *******************************************************************************/
// void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
//     IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
// }

// /**************************实现函数********************************************
// *函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
// *功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
// *******************************************************************************/
// void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
//     IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
// }

// /**************************实现函数********************************************
// *函数原型:		void MPU6050_initialize(void)
// *功　　能:	    初始化 	MPU6050 以进入可用状态。
// *******************************************************************************/
// void MPU6050_initialize(void) {
//     MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO); //设置时钟
//     MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-1000度每秒
//     MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//加速度度最大量程 +-2G
//     MPU6050_setSleepEnabled(0); //进入工作状态
// 	MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
//     MPU6050_setI2CBypassEnabled(0);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
// }




// /**************************************************************************
// 函数功能：MPU6050内置DMP的初始化
// 入口参数：无
// 返回  值：无
// 作    者：平衡小车之家
// **************************************************************************/
// void DMP_Init(void)
// { 
//    u8 temp[1]={0};
//    i2cRead(0x68,0x75,1,temp);
// 	 printf("mpu_set_sensor complete ......\r\n");
// 	if(temp[0]!=0x68)NVIC_SystemReset();
// 	if(!mpu_init())
//   {
// 	  if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
// 	  	 printf("mpu_set_sensor complete ......\r\n");
// 	  if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
// 	  	 printf("mpu_configure_fifo complete ......\r\n");
// 	  if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
// 	  	 printf("mpu_set_sample_rate complete ......\r\n");
// 	  if(!dmp_load_motion_driver_firmware())
// 	  	printf("dmp_load_motion_driver_firmware complete ......\r\n");
// 	  if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
// 	  	 printf("dmp_set_orientation complete ......\r\n");
// 	  if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
// 	        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
// 	        DMP_FEATURE_GYRO_CAL))
// 	  	 printf("dmp_enable_feature complete ......\r\n");
// 	  if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
// 	  	 printf("dmp_set_fifo_rate complete ......\r\n");
// 	  run_self_test();
// 	  if(!mpu_set_dmp_state(1))
// 	  	 printf("mpu_set_dmp_state complete ......\r\n");
//   }
// }
// /**************************************************************************
// 函数功能：读取MPU6050内置DMP的姿态信息
// 入口参数：无
// 返回  值：无
// 作    者：平衡小车之家
// **************************************************************************/
// void Read_DMP(float *Pitch,float *Roll,float *Yaw)
// {	
// 	  unsigned long sensor_timestamp;
// 		unsigned char more;
// 		long quat[4];

// 				dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);		
// 				if (sensors & INV_WXYZ_QUAT )
// 				{    
// 					q0=quat[0] / q30;
// 					q1=quat[1] / q30;
// 					q2=quat[2] / q30;
// 					q3=quat[3] / q30;
// 					*Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	
// 					*Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
// 				    *Yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw

// 				}

// }

// /**************************************************************************
// 函数功能：读取MPU6050内置温度传感器数据
// 入口参数：无
// 返回  值：摄氏温度
// 作    者：平衡小车之家
// **************************************************************************/
// int Read_Temperature(void)
// {	   
// 	  float Temp;
// 	  Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
// 		if(Temp>32768) Temp-=65536;
// 		Temp=(36.53+Temp/340)*10;
// 	  return (int)Temp;
// }
// /**************************************************************************
// 函数功能：外部中断初始化
// 入口参数：无
// 返回  值：无 
// **************************************************************************/
// void MPU6050_INT_Ini(void)
// {  
// 		GPIO_InitTypeDef GPIO_InitStructure;
// 		EXTI_InitTypeDef EXTI_InitStructure;
// 		NVIC_InitTypeDef NVIC_InitStructure;
// 		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟
// 		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PB端口时钟
// 		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	            //端口配置
// 		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
// 		GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB 
//   	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5);
	
//   	EXTI_InitStructure.EXTI_Line=EXTI_Line5;
//   	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
//   	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
//   	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//   	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
// 		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键所在的外部中断通道
//   	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
//   	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1
//   	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
//   	NVIC_Init(&NVIC_InitStructure); 
// }
// //------------------End of File----------------------------