#include "Shoot_Task.h"

void Shoot_Task(void *pvParameters){
	static portTickType currentTime;	 
	for(;;){
		currentTime = xTaskGetTickCount(); 
		ShootCtrl_Decide();
		Detect_Shoot();
		ShootRef_Set();
		Shoot_Console();
		Shoot_Send();
		Shoot_SendDown();		
		vTaskDelayUntil (&currentTime,1);
	}
}
