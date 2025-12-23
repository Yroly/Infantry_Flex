#pragma once
#include <cstdint>
#include <cstring>
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

class Chassis_Class{
public:
	ChassisSpeed_Ref_t chassis_ref;
	MotorSpeed_Ref_t motor_ref;
	
	void speed_clean(ChassisSpeed_Ref_t *ref);
	void speed_get(MotorSpeed_Ref_t *motor,ChassisSpeed_Ref_t *ref);
private:
	
};
extern Chassis_Class Chassis;