#include "dm_motor.h"

DM4310_TypeDef GimYaw,GimPitch,UnderPitch;

int float_to_uint(float x_float, float x_min, float x_max, int bits){
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits){
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
void DM_Motor_Init(DM_Motor_t *motor,uint16_t id,uint16_t mode){
	motor->mode = mode;
	motor->para.id = id;
}
void DM_Motor_read(DM_Motor_t *motor,uint8_t *rx_data){
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int,P_MIN,P_MAX, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int,V_MIN,V_MAX, 12); // (-30.0,30.0)
	motor->para.tor = uint_to_float(motor->para.t_int,T_MIN,T_MAX, 12); // (-10.0,10.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}
void DM4310_Receive(DM4310_TypeDef *Dst, uint8_t *Data){
    Dst->MchanicalAngle = ((uint16_t)Data[0]) << 8 | Data[1];
    Dst->Speed = (int16_t)(Data[2] << 8 | Data[3]) / 1000;
    Dst->TorqueCurrent = (int16_t)(Data[4] << 8 | Data[5])/1000;
    Dst->temp = (int16_t)Data[6];
    Dst->PCBtemp = (int16_t)Data[7];

    int16_t diff = Dst->MchanicalAngle - Dst->LsatAngle;
    if (diff > 4096)
        Dst->r--;
    if (diff < -4096)
        Dst->r++;

    Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle;
    Dst->Angle_DEG = Dst->Angle * 0.0439453125f;
//    Dst->Power = GetChassisMotorPower(Dst->Speed, Dst->TorqueCurrent, &Dst->PowerCOF);
    Dst->LsatAngle = Dst->MchanicalAngle;
}
/**
*@brief 0 enable
*@brief 1 disable
*@brief 2 set zero
*@brief 3 clear error
*/
HAL_StatusTypeDef DM_Motor_Ctrl(CAN_HandleTypeDef *hcan,uint16_t motor_id,uint16_t mode_id,uint8_t mode){
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	switch(mode){
		case 0 :	data[7] = 0xFC; break;//使能
		case 1 :  data[7] = 0xFD; break;//失能
		case 2 :  data[7] = 0xFE; break;//零点
		case 3 :  data[7] = 0xFB; break;//错误
		default : break;
	}
	return CAN_Send_StdDataFrame(hcan, id, data);
}
HAL_StatusTypeDef mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq){
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
	
	return CAN_Send_StdDataFrame(hcan, id, data);
}
HAL_StatusTypeDef pos_speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel)
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
	
	return CAN_Send_StdDataFrame(hcan, id, data);
}
HAL_StatusTypeDef speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float vel){
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + VEL_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	return CAN_Send_StdDataFrame(hcan, id, data);
}

HAL_StatusTypeDef DM_MotorSend(CAN_HandleTypeDef *hcan, uint32_t StdId, int16_t *Data){
    uint8_t temp[8];
    temp[0] = (uint8_t)(Data[0]);
    temp[1] = (uint8_t)(Data[0] >> 8);
    temp[2] = (uint8_t)(Data[1] );
    temp[3] = (uint8_t)(Data[1] >> 8);
    temp[4] = (uint8_t)(Data[2] );
    temp[5] = (uint8_t)(Data[2] >> 8);
    temp[6] = (uint8_t)(Data[3] );
    temp[7] = (uint8_t)(Data[3] >> 8);
    return CAN_Send_StdDataFrame(hcan, StdId, temp);
}
