#pragma once

#include "Variate.h"
#include "RMLibHead.h"

RMLIB_CPP_BEGIN
extern void ChassisInit();
extern void ChassisCtrl_Decide();
extern void ChassisRef_Update();
extern void Chassis_Offset();
extern void ChassisDown_Send();
extern void Chassis_RC_Ctrl();
extern void Chassis_Key_Ctrl();
extern void Chassis_Stop();
extern void Chassis_Close();
extern void ChassisDown_Send();

enum Chassis_Status{
	Chassis_Disable = 0,
	Chassis_Enable,
};
/**
 * @brief 矢量速度结构体
 */
typedef struct {
  int16_t forward_back_ref;  //!<@brief 前进速度
  int16_t left_right_ref;    //!<@brief 左右速度
	int16_t rotate_ref;        //!<@brief 旋转速度
} ChassisSpeed_Ref_t;

typedef struct{
	ChassisSpeed_Ref_t Chassis_Speed; //!< @brief 期望速度
	uint8_t Close_flag;				  		//!< @brief 启动标志位
	uint8_t Shift_flag;        		//!< @brief Shift加速
} Communication_Speed_t;
// up to under 结构体
typedef struct  {
	int16_t vx;            // X 方向速度（通常为归一/标定后的速度量）
	int16_t vy;            // Y 方向速度（通常为归一/标定后的速度量）
	int16_t rotate;        // 旋转速度（角速度，单位按系统约定：如 deg/s 或 rpm 映射值）
	uint8_t Close_flag;    // 底盘关闭标志（1=关闭/停止）
	uint8_t Shift_flag;    // Shift 功能键标志（1=按下）
} Gimbal_board_send_t;
/**
 * @brief 云台角度结构体
 */
typedef struct {
    float Pitch;               //!< @brief Pitch
    float Yaw;                 //!< @brief Yaw
} PTZAngle_Ref_t;

/**
 * @brief 轮组速度结构体
 */
typedef struct {
	int16_t speed_1;    //!< @brief 电机1速度
	int16_t speed_2;    //!< @brief 电机2速度
	int16_t speed_3;    //!< @brief 电机3速度
	int16_t speed_4;    //!< @brief 电机4速度
} Chassis_Motor_Speed;

/**
 * @brief 清零预期速度
 * @param[out] ref 矢量速度结构体
 */
void ChassisMotorSpeed_clean(ChassisSpeed_Ref_t *ref);
/**
 * @brief 速度矢量计算
 * @param[out] motor 轮组速度结构体
 * @param[in] ref 矢量速度结构体
 */
void ChassisMotorSpeed_get(Chassis_Motor_Speed *motor, ChassisSpeed_Ref_t *ref);

RMLIB_CPP_END
