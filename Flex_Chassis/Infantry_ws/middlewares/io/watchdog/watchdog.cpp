#include "WatchDog.h"
#include "remote_task.h"
#include "pid.h"
#include "Chassis.h"
#include "Joint.h"


WatchDog dog;

static WatchDogP List[WatchDoglength];

osStatus_t REMOTE_IfDataError( void ){
if ((remote.rc.s[0] != 1 && remote.rc.s[0] != 3 && remote.rc.s[0] != 2)
 || (remote.rc.s[1] != 1 && remote.rc.s[1] != 3 && remote.rc.s[1] != 2)
 || (remote.rc.ch[0] > 660 || remote.rc.ch[0] < -660)
 || (remote.rc.ch[1] > 660 || remote.rc.ch[1] < -660)
 || (remote.rc.ch[2] > 660 || remote.rc.ch[2] < -660)
 || (remote.rc.ch[3] > 660 || remote.rc.ch[3] < -660) )
	return osError;
else
  return osOK;
}
/*!@brief 看门狗长度*/
static uint16_t Len = 0;

void WatchDog::polling(){
	for (uint8_t i = 0; i < Len; ++i) {
		List[i]->Life++;
		if (List[i]->Life > List[i]->Max) {
			WatchBack(List[i]);
		}
	}
	osDelay(15);
}
void WatchDog::init(WatchDogP handle,uint32_t life) {
	if (Len >= WatchDoglength) return;
	handle->Max = life;
	List[Len++] = handle;
}
void WatchDog::feed(WatchDogP handle) {
	handle->Life = 0;
	FeedBack(handle);
}
void WatchDog::FeedBack(WatchDogP handle){
	if(IS_Dog(handle,Remote_Dog)){
		if(REMOTE_IfDataError() == osError){
			Remote_Dog.State = Device_Error;
		} else {
			Remote_Dog.State = Device_Online;
		}		
	}
	if(IS_Dog(handle,Chassis_Dog[0])){
		if(Chassis_Dog[0].State == Device_Error) {
			PID.Init(&Chassis.chassis_pid.motor[0],5,0,0,0,0);
		}else{
			 Chassis_Dog[0].State = Device_Online;
		}
	}
	if(IS_Dog(handle,Chassis_Dog[1])){
		if(Chassis_Dog[1].State == Device_Error) {
			PID.Init(&Chassis.chassis_pid.motor[1],5,0,0,0,0);
		}else{
			 Chassis_Dog[1].State = Device_Online;
		}
	}
	if(IS_Dog(handle,dog.Chassis_Dog[2])){
		if(Chassis_Dog[2].State == Device_Error) {
			PID.Init(&Chassis.chassis_pid.motor[2],5,0,0,0,0);
		}else{
			 Chassis_Dog[2].State = Device_Online;
		}
	}
	if(IS_Dog(handle,Chassis_Dog[3])){
		if(Chassis_Dog[3].State == Device_Error) {
			PID.Init(&Chassis.chassis_pid.motor[3],5,0,0,0,0);
		}else{
			 Chassis_Dog[3].State = Device_Online;
		}
	}
	if(IS_Dog(handle,Joint_Dog[0])){
		if(Joint_Dog[0].State == Device_Error) {
			Joint.init(&Unitree.cmd[0],1,0.38,0,6.05,2.00,0.19);
		}else{
			Joint_Dog[0].State = Device_Online;
		}
	}
	if(IS_Dog(handle,Joint_Dog[1])){
		if(Joint_Dog[1].State == Device_Error) {
			Joint.init(&Unitree.cmd[1],2,-0.38,0,2.67,2.00,0.19);
		}else{
			Joint_Dog[1].State = Device_Online;
		}
	}
}
void WatchDog::WatchBack(WatchDogP handle){
	if(IS_Dog(handle,Remote_Dog)){
		Remote_Dog.State = Device_Error;
	}
	if(IS_Dog(handle,Chassis_Dog[0])){
		PID.Init(&Chassis.chassis_pid.motor[0],0,0,0,0,0);
		Chassis_Dog[0].State = Device_Error;
	}
	if(IS_Dog(handle,Chassis_Dog[1])){
		PID.Init(&Chassis.chassis_pid.motor[1],0,0,0,0,0);
		Chassis_Dog[1].State = Device_Error;
	}
	if(IS_Dog(handle,Chassis_Dog[2])){
		PID.Init(&Chassis.chassis_pid.motor[2],0,0,0,0,0);
		Chassis_Dog[2].State = Device_Error;
	}
	if(IS_Dog(handle,Chassis_Dog[3])){
		PID.Init(&Chassis.chassis_pid.motor[3],0,0,0,0,0);
		Chassis_Dog[3].State = Device_Error;
	}
	/*joint*/
	if(IS_Dog(handle,dog.Joint_Dog[0])){
		Joint.init(&Unitree.cmd[0] , 1, 0, 0, 0, 0, 0);
		Joint_Dog[0].State = Device_Error;
	}	
	if(IS_Dog(handle,dog.Joint_Dog[1])){
		Joint.init(&Unitree.cmd[1] , 2, 0, 0, 0, 0, 0);
		Joint_Dog[1].State = Device_Error;
	}
}
