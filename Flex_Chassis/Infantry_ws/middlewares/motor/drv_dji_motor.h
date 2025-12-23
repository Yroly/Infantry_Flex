#pragma once

#include <cstdint>
#include "fdcan.h"
#include "bsp_can.h"

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
	int16_t rotatespeed;
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
}RM3508_TypeDef;

class RM_Motor_Class{
public:
	RM3508_TypeDef motor[4];
      // 根据CAN ID获取对应电机的measure指针
	void motor_read(DjiMotor_t *motor,uint8_t *data);
	void motor_ctrl(hfdcan_t *hfdcan,uint16_t id,int16_t motorl,int16_t motorr);
private:
};

extern RM_Motor_Class RM_Class;
