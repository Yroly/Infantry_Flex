#include "Chassis_Task.h"

void Chassis_Task(){
	static portTickType currentTime;
	ChassisInit();
	for(;;){
    currentTime = xTaskGetTickCount();
    ChassisCtrl_Decide();
    ChassisRef_Update();
    Chassis_Offset();
    ChassisDown_Send();
    vTaskDelayUntil(&currentTime, 1);		 
	 }
}