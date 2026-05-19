#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__

#include "can.h"
#include "CANDrive.h"
 
#define MIT_MODE 0x000
#define POS_MODE 0x100
#define VEL_MODE 0x200

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

typedef struct {
    uint16_t MchanicalAngle;    //!<@brief 机械角度
    int16_t Speed;              //!<@brief 转速
    int16_t TorqueCurrent;      //!<@brief 转矩电流
    uint8_t temp;                //!<@brief 温度
    uint8_t PCBtemp;            //!<@brief PCB温度
    uint16_t LsatMechAngle;     //!<@brief 上一次的机械角度
    int16_t r;                  //!<@brief 圈数
    int32_t ContinueMechAngle;  //!<@brief 连续化机械角度 @warning 由于启动时角度不确定，启动时连续化角度可能有一圈的偏差
    float Angle_DEG;            //!<@brief 连续化角度制角度 @warning 由于启动时角度不确定，启动时连续化角度可能有一圈的偏差
}DM4310_TypeDef;
  
  
typedef struct{
	int id;                // 电机内部设置的 can id
	int state;             // 电机状态
	int p_int;             // 整型位置信息
	int v_int;             // 整型速度信息
	int t_int;             // 整型扭矩信息
	int kp_int;            // 整型Kp信息
	int kd_int;            // 整型Kd信息
	float pos;             // 最终解析出来的位置信息  (rad)
	float vel;             // 最终解析出来的速度信息  (rad/s)
	float tor;             // 最终解析出来的扭矩信息
	float Kp;              // 最终解析出来的Kp数据
	float Kd;              // 最终解析出来的Kd数据
	float Tmos;            // 板子MOS温度
	float Tcoil;           // 电机线圈温度
}DM_motor_fdpara_t;
typedef struct{
    float P_Ref;
    float V_Ref;
    float KP_Ref;
    float KD_Ref;
    float T_ff;
}DM_Ref_t;
typedef struct{
	DM_motor_fdpara_t para;
	DM_Ref_t ref;
	uint16_t mode;
}DM_Motor_t;

extern DM4310_TypeDef GimYaw,GimPitch;

float uint_to_float(int x_int,float x_min,float x_max,int bits);
int float_to_uint(float x_float,float x_min,float x_max,int bits);
void DM_Motor_Init(DM_Motor_t *motor,uint16_t id,uint16_t mode);
void DM_Motor_read(DM_Motor_t *motor,uint8_t * rx_data);
void DM4310_Receive(DM4310_TypeDef *Dst, uint8_t *Data);
HAL_StatusTypeDef DM_MotorSend(CAN_HandleTypeDef *hcan, uint32_t StdId, int16_t *Data);
HAL_StatusTypeDef DM_Motor_Ctrl(CAN_HandleTypeDef *hcan,uint16_t motor_id,uint16_t mode_id,uint8_t mode);
HAL_StatusTypeDef mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
HAL_StatusTypeDef pos_speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel);
HAL_StatusTypeDef speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float vel);
#endif