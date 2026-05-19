#include "Chassis.h"

Chassis_Class Chassis;
void Chassis_Class::init(){
	PID.Init(&Pid.Wheel_Motor[0],10.0f,0.0f,0.0f,0.0f,0.0f);
	PID.Init(&Pid.Wheel_Motor[1],10.0f,0.0f,0.0f,0.0f,0.0f);
	PID.Init(&Pid.Wheel_Motor[2],10.0f,0.0f,0.0f,0.0f,0.0f);
	PID.Init(&Pid.Wheel_Motor[3],10.0f,0.0f,0.0f,0.0f,0.0f);

	memset(TX_Msg,0,sizeof(TX_Msg));	
	DM.Init(&Joint_Motor[0],1,MIT_MODE);
	DM.Init(&Joint_Motor[1],2,MIT_MODE);
	DM.Func_Cmd(&hfdcan2,Joint_Motor[0].para.id,Joint_Motor[0].mode,1);
	DM.Func_Cmd(&hfdcan2,Joint_Motor[1].para.id,Joint_Motor[1].mode,1);
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
			case 1 : Mode = Chassis_Mode_e::ChassisNormal ; break;
			case 3 : Mode = Chassis_Mode_e::ChassisGyroscope ; break;
			case 2 : Mode = Chassis_Mode_e::ChassisStop ; break;
		}
	}
}
void Chassis_Class::key_ctrl(){
}
void Chassis_Class::stop(){
	memset(&ChassisRefSpd,0,sizeof(ChassisRefSpd));
	memset(&Joint_Motor[0].Ref,0,sizeof(Joint_Motor[0].Ref));	
	memset(&Joint_Motor[1].Ref,0,sizeof(Joint_Motor[1].Ref));	
}
void Chassis_Class::Control(){
	if(Joint_Motor[0].para.state != 1){
		DM.Func_Cmd(&hfdcan2,Joint_Motor[0].para.id,Joint_Motor[0].mode,4);
		DM.Func_Cmd(&hfdcan2,Joint_Motor[0].para.id,Joint_Motor[0].mode,1);
	}
	if(Joint_Motor[1].para.state != 1){
		DM.Func_Cmd(&hfdcan2,Joint_Motor[1].para.id,Joint_Motor[1].mode,4);
		DM.Func_Cmd(&hfdcan2,Joint_Motor[1].para.id,Joint_Motor[1].mode,1);	
	}

	switch(Mode){
		case Chassis_Mode_e::ChassisNormal : 
				ChassisRefSpd.forward_back_ref = remote.Key_ch[1] * 5000.0f ;
				ChassisRefSpd.left_right_ref = remote.Key_ch[0] * 5000.0f;
				ChassisRefSpd.rotate_ref = ramp(remote.Key_ch[2] * 3000.0f,ChassisRefSpd.rotate_ref,10);
//				Joint_Motor[0].Ref.P_Ref = remote.Key_ch[3] * 1.0f;
//				Joint_Motor[1].Ref.P_Ref = -remote.Key_ch[3] * 1.0f;
				Joint_Motor[0].Ref = {
					.P_Ref = remote.Key_ch[3] * 1.0f,
					.V_Ref = 0.0f,
					.KP_Ref = 15.0f,
					.KD_Ref = 1.0f,
					.Tor_Ref = 7.95f
				};
				Joint_Motor[1].Ref = {
					.P_Ref = -remote.Key_ch[3] * 1.0f,
					.V_Ref = 0.0f,
					.KP_Ref = 15.0f,
					.KD_Ref = 1.0f,
					.Tor_Ref = -7.18f
				};
			break;
		case Chassis_Mode_e::ChassisGyroscope : 
				memset(&ChassisRefSpd,0,sizeof(ChassisRefSpd));
				memset(&MotorRefSpd,0,sizeof(MotorRefSpd));
				memset(&Joint_Motor[0].Ref,0,sizeof(Joint_Motor[0].Ref));	
				memset(&Joint_Motor[1].Ref,0,sizeof(Joint_Motor[1].Ref));	
			break;
		case Chassis_Mode_e::ChassisFllow :  ;
				memset(&ChassisRefSpd,0,sizeof(ChassisRefSpd));
				memset(&MotorRefSpd,0,sizeof(MotorRefSpd));
				memset(&Joint_Motor[0].Ref,0,sizeof(Joint_Motor[0].Ref));	
				memset(&Joint_Motor[1].Ref,0,sizeof(Joint_Motor[1].Ref));	
			break;
		case Chassis_Mode_e::ChassisStop :
				memset(&ChassisRefSpd,0,sizeof(ChassisRefSpd));
				memset(&MotorRefSpd,0,sizeof(MotorRefSpd));
				memset(&Joint_Motor[0].Ref,0,sizeof(Joint_Motor[0].Ref));	
				memset(&Joint_Motor[1].Ref,0,sizeof(Joint_Motor[1].Ref));	
			break;	
	}
}
void Chassis_Class::Control_loop(){
	for(int i = 0;i < 4;i ++){
		PID.Calc(&Pid.Wheel_Motor[i],Wheel_Motor[i].measure.rotatespd,MotorRefSpd.Motor[i]);
		limitmax(Pid.Wheel_Motor[i].out,RM.lim.M3508);
		TX_Msg[i] = Pid.Wheel_Motor[i].out;
	}
}
void Chassis_Class::Can_Send(){

#if CHASSIS_RUN
	#if WHEEL_RUN
	RM.motor_ctrl(&hfdcan1,0x200,TX_Msg);
	#endif
	#if JOINT_RUN
	DM.mit_ctrl(&hfdcan2,Joint_Motor[0].para.id,Joint_Motor[0].Ref.P_Ref,Joint_Motor[0].Ref.V_Ref,Joint_Motor[0].Ref.KP_Ref,Joint_Motor[0].Ref.KD_Ref,Joint_Motor[0].Ref.Tor_Ref);
	osDelay(1);
	DM.mit_ctrl(&hfdcan2,Joint_Motor[1].para.id,Joint_Motor[1].Ref.P_Ref,Joint_Motor[1].Ref.V_Ref,Joint_Motor[1].Ref.KP_Ref,Joint_Motor[1].Ref.KD_Ref,Joint_Motor[1].Ref.Tor_Ref);
	osDelay(1);
	#endif
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
    motor->Motor[0] = ref->forward_back_ref + ref->left_right_ref + ref->rotate_ref;
    motor->Motor[1] = -ref->forward_back_ref + ref->left_right_ref + ref->rotate_ref;
    motor->Motor[2] = -ref->forward_back_ref - ref->left_right_ref + ref->rotate_ref;
    motor->Motor[3] = ref->forward_back_ref - ref->left_right_ref + ref->rotate_ref;
}