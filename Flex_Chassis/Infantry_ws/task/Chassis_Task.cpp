#include "Chassis_Task.h"

extern "C" void Chassis_Task(){
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;){
		Chassis.Control();
		Chassis.speed_get(&Chassis.motor_ref,&Chassis.chassis_ref);
		Chassis.Control_loop();
		Chassis.Can_Send();
	}
}