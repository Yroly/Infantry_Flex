#include "drv_dji_motor.h"

#include <cstdint>

RM_Motor_Class RM_Class;

void RM_Motor_Class::motor_read(DjiMotor_t *motor,uint8_t *data){
	motor->mechangle = (uint16_t)(data[0] << 8 | data[1]);
	motor->rotatespd = (int16_t)(data[2] << 8 | data[3]);
	motor->current = (uint16_t)(data[4] << 8 | data[5]);
	motor->temp = data[6];
	motor->err =  static_cast<motor_err>(data[7]);
	int16_t diff = motor->mechangle - motor->lastmechangle;
		if (diff > 4096)
				motor->r--;
		if (diff < -4096)
				motor->r++;
	motor->continuemechangle = motor->r * 8192 + motor->mechangle;
	motor->lastmechangle = motor->mechangle;
	motor->angle = motor->continuemechangle * 0.0439453125f;
	motor->torque = (float)(motor->current*0.0000244140625f); 
}
void RM_Motor_Class::motor_ctrl(FDCAN_HandleTypeDef *hfdcan,uint16_t id,int16_t *Data){
	uint8_t temp[8];

	temp[0] = (uint8_t)(Data[0] >> 8);
	temp[1] = (uint8_t)(Data[0] & 0xff);
	temp[2] = (uint8_t)(Data[1] >> 8);
	temp[3] = (uint8_t)(Data[1] & 0xff);
	temp[4] = (uint8_t)(Data[2] >> 8);
	temp[5] = (uint8_t)(Data[2] & 0xff);
	temp[6] = (uint8_t)(Data[3] >> 8);
	temp[7] = (uint8_t)(Data[3] & 0xff);

	canx_send_data(hfdcan,id,temp,8);
}

