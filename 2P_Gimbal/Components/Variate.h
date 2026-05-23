#ifndef __VARIATE_H
#define __VARIATE_H
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "tim.h"
#include "remote_control.h"
#include "remote.h"
#include "motor.h"
#include "CANDrive.h"
#include "WatchDog.h"
#include "PID.h"
#include "Data_Exchange.h"

#include "stdbool.h"
#include "Attribute_Typedef.h"
#include "Function.h"
#include "ins_task.h"
#include "dm_motor.h"
#include "VT03.h"

#define CHASSIS_RUN 1
#define GIMBAL_RUN  1
#define SHOOT_RUN   1
#define current_to_out 16384.0f / 3.0f

/* speed*2*r*60 */
#define SHOOT_SPEED 6200
#define PLUCK_SPEED 4000
#define PLUCK_MOTOR_ONE 1080 //单发弹丸机械角度

#define Yaw_Mid_Front 1368
#define Yaw_Goal 637
#define Pitch_Mid    3963
#define UnderPitch_Mid 6868

#define P_ADD_limit 6
#define P_LOSE_limit -50
    
#define UNP_ADD_limit -76
#define UNP_LOSE_limit -176

#define TUNNEL_MID_YAW 28

#define limit(IN, MAX, MIN) \
    if (IN < MIN)           \
        IN = MIN;           \
    if (IN > MAX)           \
        IN = MAX
    
#if (Yaw_Mid_Front + 2048) > 8191
#define Yaw_Mid_Left Yaw_Mid_Front - 6143
#else
#define Yaw_Mid_Left Yaw_Mid_Front + 2048
#endif

#if (Yaw_Mid_Left + 2048) > 8191
#define Yaw_Mid_Back Yaw_Mid_Left - 6143
#else
#define Yaw_Mid_Back Yaw_Mid_Left + 2048
#endif

#if (Yaw_Mid_Back + 2048) > 8191
#define Yaw_Mid_Right Yaw_Mid_Back - 6143
#else
#define Yaw_Mid_Right Yaw_Mid_Back + 2048
#endif

enum{
	PITCH = 0,
	YAW = 1,
	UnderP = 2,
	GIMBAL_SUM = 3,
};
enum{
  LEFT = 1,
  RIGHT= 0,
  FRIC_SUM = 2,
};
/** @brief 电机参考速度 */
#define STD_Speed  2406		// 标速1m/s
#define STD_Omega  6142		// 标准1rpm/s
#define STD_Angle  0.36f	// 角度制1rpm/s
#define STD_MAngle 8.192	// 机械角度1rpm/s

/* 卡弹标志位 */
extern int StuckFlag;
/* 云台初始化标志位*/
extern uint8_t GimbalInitFlag;

/* 系统状态 */
typedef enum{
	SYSTEM_STARTING = 0,     //!< @brief 正在启动
	SYSTEM_RUNNING  = 1,	 //!< @brief 正在运行
} eSystemState;
/* 设备状态 */
typedef enum{
	Device_Offline = 0,     //!< @brief 设备离线
	Device_Online  = 1,	    //!< @brief 设备在线
	Device_Error   = 2	    //!< @brief 设备错误
} eDeviceState;
typedef struct {
	eDeviceState Remote_State,IMU_State, Gimbal_State[GIMBAL_SUM],UnderP_State, Shoot_State[FRIC_SUM], Pluck_State, Down_State, PC_State,Referee_State,VT03_State;
}DeviceStates;
extern DeviceStates DeviceState;
extern eSystemState SystemState;

/* 电机 */
extern RM3508_TypeDef Shoot_Motor[FRIC_SUM];
extern M2006_TypeDef  Pluck_Motor;

/* 云台归中位置 */
typedef enum{
	FRONT = 0,       //!< @brief 前
	BACK  = 1,       //!< @brief 后
} eMidMode;
extern eMidMode MidMode;

/** 
 * @breief 下板数据结构 
 * @param  0x101
 */
typedef struct{
	int8_t game_state;   //游戏状态 0--比赛外 1--比赛中
	int8_t robot_color;  //自身颜色 0--红色 1--蓝色
	uint16_t heat_limit; 	//热量上限
	uint16_t heat_cooling;//冷却值
	uint16_t heat_now;    //当前热量
} Chassis_RefereeMsg_t;
/* 0x102 */
typedef struct{
	float Chassis_Speed; //底盘角速度
	float bullet_speed;//弹速
} Chassis_Msg_t;
/* 发送底盘数据 */
typedef enum {
	Gimbal_offline = 0,    
	Gimbal_online  = 1,     
}Gimbal_status_e;
typedef enum {
	shoot_offline = 0,     
	shoot_online  = 1,      
}shoot_status_e;
typedef enum {
	stop = 0,    
	normal  = 1,   
  rotate = 2,
  Auto = 3,
  follow = 4
}move_status_e;
typedef enum {
	vision_offline = 0,   
	vision_online  = 1,    
}vision_status_e;
typedef enum{
	shoot_mode_stop = 0,   
	shoot_mode_ready  = 1,   
  shoot_mode_fire = 2,
  shoot_mode_follow = 3,
  shoot_mode_stucking = 4,
}shoot_mode_e;
typedef struct  {
  Gimbal_status_e Pitch: 1;
  Gimbal_status_e Yaw : 1;
	Gimbal_status_e UnderPitch : 1;
} Gimbal_status_t;
typedef struct  {
	int16_t vision_distance;  
	int16_t Pitch_angle;
	int16_t UnderPitch_angle;
	uint16_t Offset_Angle;
} Gimbal_data_t;
typedef struct{
	Gimbal_status_t Gimbal_status;         
	shoot_status_e shoot_status : 1;          
	move_status_e move_status : 4;     
	vision_status_e vision_status : 1;          
	shoot_mode_e shoot_mode;				           
	uint8_t Key; 
	uint8_t goal_flag; 
}Gimbal_action_t;
/* 上下板通信发送 */
extern Gimbal_data_t Gimbal_data;
extern Gimbal_action_t Gimbal_action;
/* 裁判系统数据 */
extern Chassis_RefereeMsg_t Referee_data_Rx;      // 上下板通信接收数据
extern Chassis_Msg_t Chassis_data_Rx;
/* 任务句柄 */
extern TaskHandle_t  Chassis_Task_handle,Gimbal_Task_handle,Shoot_Task_handle, Ins_Task_handle,Music_Task_handle,usb_task_handle;
/* 看门狗 */
extern WatchDog_TypeDef Remote_Dog, IMU_Dog, Gimbal_Dog[GIMBAL_SUM],UnderP_Dog, Shoot_Dog[FRIC_SUM], Pluck_Dog, Down_Dog, PC_Dog,Referee_Dog,VT03_Dog;

extern uint8_t NormalModeFlag,GyroscopeModeFlag;
#endif
