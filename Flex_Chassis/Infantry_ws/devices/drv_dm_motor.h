#pragma once

#include "main.h"
#include "fdcan.h"
#include "bsp_can.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define T_MIN -54.0f
#define T_MAX 54.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
typedef struct {
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float POS;
	float VEL;
	float Torque;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}DM_motor_fbpara_t;
typedef struct{
	float P_Ref;
	float V_Ref;
	float KP_Ref;
	float KD_Ref;
	float Tor_Ref;
}MotorRef_t;
typedef struct{
	DM_motor_fbpara_t para;
	uint16_t mode;
	MotorRef_t Ref;
}Joint_Motor_t;
class DM_Motor_Class{
public:

void HandleRx(DM_motor_fbpara_t *motor,uint8_t *rx_data,uint32_t datalen);
void Func_Cmd(FDCAN_HandleTypeDef *hfdcan,uint16_t motor_id,uint16_t mode_id,uint16_t func_id);

void mit_ctrl(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
void pos_speed_ctrl(FDCAN_HandleTypeDef* hfdcan,uint16_t motor_id, float pos, float vel);
void speed_ctrl(FDCAN_HandleTypeDef* hfdcan,uint16_t motor_id, float _vel);

void Init(Joint_Motor_t *motor,uint16_t id,uint16_t mode);
	
float Hex_To_Float(uint32_t *Byte,int num);
uint32_t FloatTohex(float HEX);

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);		
};
extern DM_Motor_Class DM;
