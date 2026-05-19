#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "Variate.h"
#include "Function.h"

typedef struct{
  float Yaw;
  float vYaw;
  float vPitch;
  float aYaw;
  float aPitch;
}KFF_t;
extern KFF_t K_FF;
typedef enum{
    INIT = 0,
    GYRO = 1,
    AIM = 2,
    GIMBAL_MODE = 3
}eGimbalPidMode;
extern eGimbalPidMode GimbalPidMode;

typedef enum {
	gNormal = 0,
	gAim    = 1,
  gStop   = 2
}eGimbalCtrl;
extern eGimbalCtrl GimbalCtrl;
typedef struct{
	int LastCtrl;
	float Ref[GIMBAL_SUM];
	float YawInit,PitchInit;
	float increase[GIMBAL_SUM];
}eGimbal;
extern eGimbal Gimbal;
extern PID_TypeDef Gimbal_Speed_pid_Yaw[GIMBAL_MODE], Gimbal_Place_pid_Yaw[GIMBAL_MODE];																						

extern void GimbalInit();//云台初始化
extern void MedianInit();//归中
extern void GimbalCtrl_Decide();//决定控制方式
extern void Gimbal_RC_Ctrl();//遥控器控制
extern void Gimbal_Key_Ctrl();//键鼠控制
extern void Gimbal_Stop();//急停

extern void GimbalRef_Update();//更新期望值
extern void Gimbal_Calc();
extern void Gimbal_Send();
extern void Gimbal_SendDown();

extern float Kff_v,Kff_a;

#endif
