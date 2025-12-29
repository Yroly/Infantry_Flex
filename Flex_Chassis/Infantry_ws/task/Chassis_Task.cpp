#include "Chassis_Task.h"

extern "C" void Chassis_Task(){
	for(int i =0;i<4;i++){
		dog.init(&dog.Chassis_Dog[i],20);
	}
	dog.init(&dog.Joint_Dog[0],20);
	dog.init(&dog.Joint_Dog[1],20);
	
	static portTickType currentTime;
	Chassis.init();
	for(;;){
		currentTime = xTaskGetTickCount();
		if(dog.Remote_Dog.State == Device_Online){
			Chassis.decide();
			Chassis.Control();
			Unitree.modify_data(&Unitree.cmd[0]);
			Unitree.modify_data(&Unitree.cmd[1]);
			Chassis.speed_get(&Chassis.motor_ref,&Chassis.chassis_ref);
			Chassis.Control_loop();
			Chassis.Can_Send();		
		}

		
		vTaskDelayUntil(&currentTime,2);
	}
}