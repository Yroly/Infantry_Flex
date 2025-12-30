#include "Chassis.h"

Chassis_Class Chassis;
void Chassis_Class::init(){
	PID.Init(&chassis_pid.motor[0],10.0f,0.0f,0.0f,0.0f,0.0f);
	PID.Init(&chassis_pid.motor[1],10.0f,0.0f,0.0f,0.0f,0.0f);
	PID.Init(&chassis_pid.motor[2],10.0f,0.0f,0.0f,0.0f,0.0f);
	PID.Init(&chassis_pid.motor[3],10.0f,0.0f,0.0f,0.0f,0.0f);
		
	memset(TX_Msg,0,sizeof(TX_Msg));
}

void Chassis_Class::decide(){
	if(dog.Remote_Dog.State == Device_Online){
		remote.RemoteMode == REMOTE_INPUT ? rc_ctrl() :
		remote.RemoteMode == KEY_MOUSE_INPUT ? key_ctrl() : stop();
	}else{
		
	}
}
void Chassis_Class::rc_ctrl(){
	if(remote.RemoteMode == REMOTE_INPUT){
		switch(remote.rc.s[0]){
			case 1 : mode = Chassis_Mode_e::ChassisNormal ; break;
			case 3 : mode = Chassis_Mode_e::ChassisGyroscope ; break;
			case 2 : mode = Chassis_Mode_e::ChassisStop ; break;
		}
	}
}
void Chassis_Class::key_ctrl(){
}
void Chassis_Class::stop(){
	memset(&chassis_ref,0,sizeof(chassis_ref));
}
void Chassis_Class::Control(){
	switch(mode){
		case Chassis_Mode_e::ChassisNormal : 
				chassis_ref.forward_back_ref = remote.Key_ch[1] * 3000.0f ;
				chassis_ref.left_right_ref = remote.Key_ch[0] * 3000.0f;
				chassis_ref.rotate_ref = ramp(remote.Key_ch[2] * 1000.0f,chassis_ref.rotate_ref,10);
			break;
		case Chassis_Mode_e::ChassisGyroscope : 
				memset(&chassis_ref,0,sizeof(chassis_ref));
				memset(&motor_ref,0,sizeof(motor_ref));
			break;
		case Chassis_Mode_e::ChassisFllow :  ;
		
			break;
		case Chassis_Mode_e::ChassisStop :
				memset(&chassis_ref,0,sizeof(chassis_ref));
				memset(&motor_ref,0,sizeof(motor_ref));
			break;	
	}
}
void Chassis_Class::Control_loop(){
	PID.Calc(&chassis_pid.motor[0],RM_Class.motor[0].measure.rotatespd,motor_ref.speed_1);
	PID.Calc(&chassis_pid.motor[1],RM_Class.motor[1].measure.rotatespd,motor_ref.speed_2);
	PID.Calc(&chassis_pid.motor[2],RM_Class.motor[2].measure.rotatespd,motor_ref.speed_3);
	PID.Calc(&chassis_pid.motor[3],RM_Class.motor[3].measure.rotatespd,motor_ref.speed_4);
	limitmax(chassis_pid.motor[0].out,RM_Class.lim.M3508);
	limitmax(chassis_pid.motor[1].out,RM_Class.lim.M3508);
	limitmax(chassis_pid.motor[2].out,RM_Class.lim.M3508);
	limitmax(chassis_pid.motor[3].out,RM_Class.lim.M3508);
	for(int i =0;i<4;i++){
		TX_Msg[i] = chassis_pid.motor[i].out;
	}
}
void Chassis_Class::Can_Send(){
#if CHASSIS_RUN
	RM_Class.motor_ctrl(&hfdcan1,0x200,TX_Msg);
#endif
}
int16_t Chassis_Class::ramp(int16_t target,int16_t measure,int16_t step){
	float buffer = target - measure;
	if (buffer > 0){
		if (buffer > step) measure += step;  
		else measure += buffer;
	}else{
		if (buffer < -step) measure += -step;
		else measure += buffer;
	}
	return measure;
}
void Chassis_Class::speed_clean(ChassisSpeed_Ref_t *ref){
	ref->forward_back_ref = 0;
	ref->left_right_ref = 0;
	ref->rotate_ref = 0;
}
void Chassis_Class::speed_get(MotorSpeed_Ref_t *motor, ChassisSpeed_Ref_t *ref){
    motor->speed_1 = ref->forward_back_ref + ref->left_right_ref + ref->rotate_ref;
    motor->speed_2 = -ref->forward_back_ref + ref->left_right_ref + ref->rotate_ref;
    motor->speed_3 = -ref->forward_back_ref - ref->left_right_ref + ref->rotate_ref;
    motor->speed_4 = ref->forward_back_ref - ref->left_right_ref + ref->rotate_ref;
}