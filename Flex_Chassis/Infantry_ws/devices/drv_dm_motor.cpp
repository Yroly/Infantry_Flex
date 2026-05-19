#include "drv_dm_motor.h"
#include "fdcan.h"
#include "arm_math.h"

DM_Motor_Class DM;

float DM_Motor_Class::Hex_To_Float(uint32_t *Byte,int num){
	return *((float*)Byte);
}

uint32_t DM_Motor_Class::FloatTohex(float HEX){
	return *( uint32_t *)&HEX;
}
int DM_Motor_Class::float_to_uint(float x_float, float x_min, float x_max, int bits){
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
float DM_Motor_Class::uint_to_float(int x_int, float x_min, float x_max, int bits){
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void DM_Motor_Class::Init(Joint_Motor_t *motor,uint16_t id,uint16_t mode){
	motor->mode = mode;
	motor->para.id = id;
}
void DM_Motor_Class::HandleRx(DM_motor_fbpara_t *motor,uint8_t *rx_data,uint32_t data_len){
	motor->id = (rx_data[0])&0x0F;
	motor->state = (rx_data[0])>>4;
	motor->p_int=(rx_data[1]<<8)|rx_data[2];
	motor->v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->POS = uint_to_float(motor->p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	motor->VEL = uint_to_float(motor->v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
	motor->Torque = uint_to_float(motor->t_int, T_MIN, T_MAX, 12);  // (-10.0,10.0)
	motor->Tmos = (float)(rx_data[6]);
	motor->Tcoil = (float)(rx_data[7]);
}
void DM_Motor_Class::Func_Cmd(FDCAN_HandleTypeDef *hfdcan,uint16_t motor_id,uint16_t mode_id,uint16_t func_id){
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	switch(func_id){
		case 1 : data[7] = 0xFC; break;//enable
		case 2 : data[7] = 0xFD; break;//disable
		case 3 : data[7] = 0xFE; break;//setzero
		case 4 : data[7] = 0xFB; break;//clearerr
	}
	canx_send_data(hfdcan, id, data, 8);	
}
void DM_Motor_Class::mit_ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq){
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hfdcan, id, data, 8);
}
void DM_Motor_Class::pos_speed_ctrl(FDCAN_HandleTypeDef* hfdcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	canx_send_data(hfdcan, id, data, 8);
}
void DM_Motor_Class::speed_ctrl(FDCAN_HandleTypeDef* hfdcan,uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_send_data(hfdcan, id, data, 4);
}