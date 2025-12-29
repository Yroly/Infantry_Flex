#pragma once

#include <cstdint>
#include "fdcan.h"
#include "bsp_can.h"
/**
* @brief 电机限幅结构体
*/
typedef struct{
	int16_t M3508;
	int16_t GM6020;
	int16_t M2006;
}Motorlim_t;
enum class motor_err : uint8_t{
	NO_ERR = 0,
	UN_ACCESS = 1,
	OVER_V = 2,
	NO_CON = 3,
	LOST_DATA = 4,
	OVER_TEMP = 5,//180
	FAILED = 7,
	OVER_HOT = 8//125
};
/**
* @brief RM电机数据结构体
 */
typedef struct{
	uint16_t mechangle;
	int16_t rotatespd;
	int16_t current;
	uint8_t temp;
	motor_err err;
	uint16_t lastmechangle;     //!<@brief 上一次的机械角度	
	int16_t r;                  //!<@brief 圈数
	int32_t continuemechangle;  //!<@brief 连续化机械角度
	float angle;            		//!<@brief 连续化角度
	float torque;      					//!<@brief 转矩
}DjiMotor_t;
/**
* @brief RM3508电机结构体 
*/
typedef struct{
	DjiMotor_t measure;
	float speed;
	int16_t give_current;
  struct PowerCOF_s {
		float ss;               //!<@brief 速度平方项系数
		float sc;               //!<@brief 速度,转矩电流乘积项系数
		float cc;               //!<@brief 转矩电流平方项系数
		float constant;         //!<@brief 常量
	} PowerCOF;               //!<@brief 计算功率所用的系数,由MATLAB拟合
}M3508_t;

class RM_Motor_Class{
public:
	M3508_t motor[4];
	Motorlim_t lim = {
		.M3508 = 6000,
		.GM6020 = 16384,
		.M2006 = 16384
	};
	void motor_read(DjiMotor_t *motor,uint8_t *data);
	void motor_ctrl(FDCAN_HandleTypeDef *hfdcan,uint16_t id,int16_t *Data);
private:
};

extern RM_Motor_Class RM_Class;
