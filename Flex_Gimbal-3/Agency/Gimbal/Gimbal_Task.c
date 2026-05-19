#include "Gimbal_Task.h"

void Gimbal_Task(){
	static portTickType currentTime;
	for(;;){
		currentTime = xTaskGetTickCount();			 
		if(SystemState != SYSTEM_RUNNING){
			if(GimbalInitFlag == 0) GimbalInit();
			MedianInit();
#if !GIMBAL_RUN
		SystemState = SYSTEM_RUNNING;
#endif				
		}else{
			GimbalCtrl_Decide();
			GimbalRef_Update();
      Gimbal_Calc();	
      Gimbal_Send();				
      vTaskDelayUntil(&currentTime, 1);		 
    }
	}
}
