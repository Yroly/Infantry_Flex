#include "Joint.h"
#include "Freertos.h"
#include "task.h"
#include "cmsis_os2.h"

Joint_Class Joint;

extern "C" void Joint_Task(){
	PID.Init(&Joint.joint_pid.pitch,10.0f,0.0f,0.0f,0.0f,0.0f);
	Joint.init(&Unitree.cmd[0],1, 0 , 0, 0, 0, 0);
	Joint.init(&Unitree.cmd[1],2, 0 , 0, 0, 0, 0);	
	
	Joint.init(&Unitree.cmd[0],1, 0.38 , 0, 6.05, 2.00, 0.19);
	Joint.init(&Unitree.cmd[1],2,-0.38 , 0, 2.67, 2.00, 0.19);

	Unitree.measure[0].init_pos = 6.05f;
	Unitree.measure[1].init_pos = 2.67f;
}
void Joint_Class::control(){
	switch(Chassis.mode){
		case Chassis_Mode_e::ChassisNormal : 
				Unitree.cmd[0].Pos = Unitree.measure[0].init_pos - joint_pid.pitch.out;
				Unitree.cmd[1].Pos = Unitree.measure[1].init_pos + joint_pid.pitch.out;
				limit(Unitree.cmd[0].Pos, Joint_Left_Pos_Max, Joint_Left_Pos_Min);
				limit(Unitree.cmd[1].Pos, Joint_Right_Pos_Max, Joint_Right_Pos_Min);
			break;
		case Chassis_Mode_e::ChassisGyroscope : 
			break;
		case Chassis_Mode_e::ChassisFllow :
			break;
		case Chassis_Mode_e::ChassisStop :
			break;		
	}
}
void Joint_Class::control_loop(){
	PID.Calc(&joint_pid.pitch,INS.Pitch,0.058);
	
}
void Joint_Class::exchange(){
#if JOINT_RUN	
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Unitree.cmd[0].motor_send_data, sizeof(Unitree.cmd[0].motor_send_data));
	HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&Unitree.cmd[1].motor_send_data, sizeof(Unitree.cmd[1].motor_send_data));

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)&Unitree.measure[0].motor_recv_data, sizeof(Unitree.measure[0].motor_recv_data));
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)&Unitree.measure[1].motor_recv_data, sizeof(Unitree.measure[1].motor_recv_data));
#endif	
}
void Joint_Class::init(MotorCmd_t *cmd, unsigned short ID, float t, float w, float pos, float k_p, float k_w){
	cmd->id = ID;
	cmd->mode = 1;
	cmd->T = t;
	cmd->W = w;
	cmd->Pos = pos;
	
	cmd->K_P = k_p;
	limit(cmd->K_P, (float)25.599, 0);
	
	cmd->K_W = k_w;
	limit(cmd->K_W, (float)25.599, 0);	
}

