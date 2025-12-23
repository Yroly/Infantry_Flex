#include "INS_Task.h"
#include "QuaternionEKF.h"
#include "mahony_filter.h"
#include "cmsis_os2.h"

INS_t INS;
struct MAHONY_FILTER_t mahony;
Axis3f Gyro,Accel;
uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time;
int stop_time;

void INS_Task(void){
	 mahony_init(&mahony,1.0f,0.0f,0.001f);
	 for(;;){
		ins_dt = DWT_GetDeltaT(&INS_DWT_Count);
    
		mahony.dt = ins_dt;

    BMI088_Read(&BMI088);

    INS.Accel[0] = BMI088.Accel[0];
    INS.Accel[1] = BMI088.Accel[1];
    INS.Accel[2] = BMI088.Accel[2];
	  Accel.x=BMI088.Accel[0];
	  Accel.y=BMI088.Accel[1];
		Accel.z=BMI088.Accel[2];
    INS.Gyro[0] = BMI088.Gyro[0];
    INS.Gyro[1] = BMI088.Gyro[1];
    INS.Gyro[2] = BMI088.Gyro[2];
  	Gyro.x=BMI088.Gyro[0];
		Gyro.y=BMI088.Gyro[1];
		Gyro.z=BMI088.Gyro[2];

		mahony_input(&mahony,Gyro,Accel);
		mahony_update(&mahony);
		mahony_output(&mahony);
	  RotationMatrix_update(&mahony);
				
		INS.q[0]=mahony.q0;
		INS.q[1]=mahony.q1;
		INS.q[2]=mahony.q2;
		INS.q[3]=mahony.q3;
   		
		if(ins_time>3000.0f){
			INS.ins_flag=1;
		  INS.Roll=mahony.roll;
      INS.Pitch=mahony.pitch;
		  INS.Yaw=mahony.yaw;			
			if (INS.Yaw - INS.YawAngleLast > 3.1415926f){
					INS.YawRoundCount--;
			}
			else if (INS.Yaw - INS.YawAngleLast < -3.1415926f){
					INS.YawRoundCount++;
			}
			INS.YawTotalAngle = 2 * PI * INS.YawRoundCount + INS.Yaw;
			INS.YawAngleLast = INS.Yaw;
		}else{
		 ins_time++;
		}		
    osDelay(1);
	}
} 