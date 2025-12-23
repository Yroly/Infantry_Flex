#include "drv_dji_motor.h"

#include <cstdint>

RM_Motor_Class RM_Class;

void RM_Motor_Class::motor_read(DjiMotor_t *motor,uint8_t *data){
	motor->mechangle = (uint16_t)(data[0] << 8 | data[1]);
	motor->rotatespeed = (int16_t)(data[2] << 8 | data[3]);
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
void RM_Motor_Class::motor_ctrl(hfdcan_t *hfdcan,uint16_t id,int16_t motorl,int16_t motorr){
	uint8_t temp[8];

	temp[0] = 0;
	temp[1] = 0;
	temp[2] = motorl >> 8;
	temp[3] =	motorl;
	temp[4] = motorr >> 8;
	temp[5] = motorr;
	temp[6] = 0;
	temp[7] = 0;

	canx_send_data(hfdcan,id,temp,8);
}

