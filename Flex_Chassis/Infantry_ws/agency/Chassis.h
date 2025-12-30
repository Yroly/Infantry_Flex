#pragma once
#include <cstdint>
#include <cstring>
#include "remote_task.h"
#include "pid.h"
#include "drv_dji_motor.h"
#include "dvc_unitree.h"
#include "INS_Task.h"
/*底盘运行状态*/
#define CHASSIS_RUN 1
//1 5.92  12.85
//2 2.77  -3.90
#define Joint_Left_Pos_Max (float)12.85  
#define Joint_Left_Pos_Min (float)5.95   
#define Joint_Right_Pos_Max (float)2.77  
#define Joint_Right_Pos_Min (float)-3.90
/*ljy*/
//#define Joint_Left_Pos_Max (float)6.85  
//#define Joint_Left_Pos_Min (float)1.57   
//#define Joint_Right_Pos_Max (float)1.75  
//#define Joint_Right_Pos_Min (float)-3.53  
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
	int16_t speed_1;    //!<@brief 底盘电机1速度
	int16_t speed_2;    //!<@brief 底盘电机2速度
	int16_t speed_3;    //!<@brief 底盘电机3速度
	int16_t speed_4;    //!<@brief 底盘电机4速度
} MotorSpeed_Ref_t;
/**
 * @brief 底盘PID结构体
 */
typedef struct{
	PidTypeDef motor[4];
}ChassisPid_t;
enum class Chassis_Mode_e : uint8_t{
	ChassisStop = 0,
	ChassisFllow = 1,
	ChassisNormal = 2,
	ChassisGyroscope = 3
};
class Chassis_Class{
public:
	ChassisSpeed_Ref_t chassis_ref;
	MotorSpeed_Ref_t motor_ref;
	ChassisPid_t chassis_pid;
	Chassis_Mode_e mode;
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