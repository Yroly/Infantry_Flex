#pragma once
#include <cstdint>
#include <cstring>
#include "uart_task.h"
#include "pid.h"
#include "drv_dji_motor.h"
#include "drv_dm_motor.h"
#include "dvc_unitree.h"
#include "INS_Task.h"
/*底盘运行状态*/
#define CHASSIS_RUN 1
#define WHEEL_RUN 1
#define JOINT_RUN 1
/**
 * @brief 矢量速度结构体
 */
typedef struct {
	int16_t forward_back_ref;  //!<@brief 前进方向速度
	int16_t left_right_ref;    //!<@brief 左右方向速度
	int16_t rotate_ref;        //!<@brief 旋转速度
} ChassisSpeed_Ref_t;
/**
 * @brief 轮组速度结构体
 */
typedef struct {
	int64_t Motor[4];
} MotorSpeed_Ref_t;
/**
 * @brief 底盘PID结构体
 */
typedef struct{
	PidTypeDef Wheel_Motor[4];
	PidTypeDef Joint_Motor[2];
}ChassisPid_t;
enum class Chassis_Mode_e : uint8_t{
	ChassisStop = 0,
	ChassisFllow = 1,
	ChassisNormal = 2,
	ChassisGyroscope = 3
};
//*	板间通信 *//
// 云台->底盘数据包
struct __PACKED RxBoard{
  int16_t vx;            // 单位 基准速度的倍率（基准速度由底盘模块根据功率自动计算）
  int16_t vy;            // 单位 基准速度的倍率
  int16_t rotate;        // 单位 旋转速度度每秒
	uint8_t Close_flag;				  //!< @brief 底盘关闭标志位
  uint8_t Shift_flag;         //!< @brief Shift跑路
};    
// 底盘->云台数据包
struct __PACKED ChassisCmd{
//    int8_t  robot_color;      // 机器人颜色
	uint16_t heat_limit_remain; 	// 剩余热量
	uint16_t heat_limit_recover; 	//冷却速率	  
	int16_t ChassisSpeed; 				//底盘角速度
	int8_t game_state_robot_color;//比赛状态 --0 未开始 --1 开始
};
			


class Chassis_Class{
public:
	M3508_t Wheel_Motor[4];
	Joint_Motor_t Joint_Motor[2];

	RxBoard RxGimbal;
	ChassisSpeed_Ref_t ChassisRefSpd;
	MotorSpeed_Ref_t MotorRefSpd;
	ChassisPid_t Pid;
	Chassis_Mode_e Mode;
	int16_t TX_Msg[4];
	void speed_clean(ChassisSpeed_Ref_t *ref);
	void speed_get(MotorSpeed_Ref_t *motor,ChassisSpeed_Ref_t *ref);
	void init();
	void decide();
	void rc_ctrl();
	void key_ctrl();
	void stop();
	void Control();
	void Control_loop();
	void Can_Send();
	int16_t ramp(int16_t target,int16_t measure,int16_t step);
private:
};
extern Chassis_Class Chassis;