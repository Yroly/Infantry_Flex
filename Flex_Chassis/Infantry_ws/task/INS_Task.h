#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "BMI088driver.h"
#include "QuaternionEKF.h"

#define INS_TASK_PERIOD 1

typedef struct
{
    float q[4]; 
    float Gyro[3];  
    float Accel[3]; 

    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
		float YawAngleLast;
		float YawRoundCount;
				
		uint8_t ins_flag;
} INS_t;

typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern void INS_Init(void);
extern void INS_task(void);
extern INS_t INS;

#endif


