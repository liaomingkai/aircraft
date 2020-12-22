#include <math.h>
#include "MPU6050.h"
#include "MPU6050DMP.h"

extern float Angle_Pitch, Angle_Roll, Angle_Yaw;
extern float GYRO_X, GYRO_Y, GYRO_Z;


#ifndef ahrs_h
#define ahrs_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

//extern volatile float twoKp;		  // 2 * proportional gain (Kp)
//extern volatile float twoKi;		  // 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
//º∆À„≈∑¿≠Ω«
void CalcuEulerAngle2(float *pitch, float *roll, float *yaw);
void get_angle();
float Get_AHRS_Time(void);
void AHRS_Time_Init(void);
#endif
