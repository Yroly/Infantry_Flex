#pragma once

#include "main.h"

/* 默认模式：110 帧接收结构体（底盘控制相关） */
typedef struct
{
    int16_t V_X;            // X 方向速度（通常为归一/标定后的速度量）
    int16_t V_Y;            // Y 方向速度（通常为归一/标定后的速度量）
    int16_t Rotate;         // 旋转速度（角速度，单位按系统约定：如 deg/s 或 rpm 映射值）
    uint8_t Close_flag;     // 底盘关闭标志（1=关闭/停止）
    uint8_t Shift_flag;     // Shift 功能键标志（1=按下）
} ChasisBoardReceive_110;

/*
        坐标系约定（底盘坐标）：
          ^ y
          |
          |
          |
          |
   <------+------> x
          |
          |
          |
          |
*/

/*********************************************************************
 * 1. 云台单轴状态枚举（Pitch / Yaw 单独在线状态）
 *********************************************************************/
typedef enum {
    Gimbal_offline = 0,    // 离线：未连接/异常/未初始化等
    Gimbal_online  = 1     // 在线：工作正常
} Gimbal_status_e;

/*********************************************************************
 * 2. 云台双轴状态结构体（位域表示 Pitch 与 Yaw 是否在线）
 *********************************************************************/
typedef struct {
    Gimbal_status_e Pitch : 1;  // Pitch 轴状态：0=离线，1=在线
    Gimbal_status_e Yaw   : 1;  // Yaw 轴状态：0=离线，1=在线
} Gimbal_status_t;

/*********************************************************************
 * 3. 射击系统状态枚举
 *********************************************************************/
typedef enum {
    shoot_offline = 0,     // 离线：故障/未初始化/未连接
    shoot_online  = 1      // 在线：工作正常
} shoot_status_e;

/*********************************************************************
 * 4. 运动状态枚举（底盘/平台运动模式）
 *********************************************************************/
typedef enum {
    stop   = 0,            // 停止
    normal = 1,            // 正常运动（平移/常规行驶）
    rotate = 2,            // 原地旋转
    rise_mid    = 3,            // 伸一半腿
    rise_high  = 4             // 伸完全腿
} move_status_e;

/*********************************************************************
 * 5. 视觉系统状态枚举
 *********************************************************************/
typedef enum {
    vision_offline = 0,    // 离线：未启动/掉线/故障
    vision_online  = 1     // 在线：工作正常
} vision_status_e;

/*********************************************************************
 * 6. 射击模式枚举（射击系统工作模式）
 *********************************************************************/
typedef enum {
    shoot_mode_stop     = 0,  // 停止/待机
    shoot_mode_ready    = 1,  // 准备射击（预上电/预热/等待指令）
    shoot_mode_fire     = 2,  // 正在射击（发射中）
    shoot_mode_follow   = 3,  // 跟随射击（跟随目标并自动/半自动发射）
    shoot_mode_stucking = 4   // 卡弹/堵转等异常状态，需要处理
} shoot_mode_e;

/*********************************************************************
 * 7. 120 帧接收结构体：综合状态与模式数据
 *********************************************************************/
typedef struct {
    Gimbal_status_t   Gimbal_status;      // 云台双轴状态（Pitch + Yaw）
    shoot_status_e    shoot_status : 1;   // 射击系统状态：0=离线，1=在线
    move_status_e     move_status  : 4;   // 运动状态：占 4 bit（0~15）
    vision_status_e   vision_status: 1;   // 视觉系统状态：0=离线，1=在线
    shoot_mode_e      shoot_mode;         // 射击模式
    uint8_t           Key;                // 按键状态汇总（如遥控器/键盘按键 bitmask）
    uint8_t           vision_number;      // 视觉识别编号/目标编号（按你的协议定义）
} ChasisBoardReceive_120;

/*********************************************************************
 * 8. 130 帧接收结构体：视觉/角度/距离相关数据
 *********************************************************************/
typedef struct  {
    uint16_t vision_distance;   // 视觉测距（单位按协议：mm / cm / dm 等）
    int16_t  Pitch_angle;       // Pitch 角度（单位按协议：0.01deg / deg 等）
    int16_t  Yaw_angle;         // Yaw 角度（单位按协议）
    uint16_t Offset_Angle;      // 偏移角（单位按协议）
} ChasisBoardReceive_130;

typedef struct {
    uint16_t heat_limit_remain;     // 剩余热量上限（或剩余热量额度，单位按裁判系统定义）
    uint16_t heat_limit_recover;    // 冷却速率/热量恢复值（单位按裁判系统定义）
	   uint8_t robot_id;                 //机器人id
	  uint8_t game_progammer;         //比赛进程
	  uint16_t Bullet_Velocity;       //弹速 将float*100，然后直接转换成uint16类型发送
//    int8_t   game_state_robot_color;// 比赛状态/机器人颜色（按协议定义组合字段）
//                                  // 例如：
//                                  //  - 比赛状态：0=未开始，1=开始（示例）
//                                  //  - 机器人颜色：红/蓝（示例）
//                                  // 具体以 你的协议为准
	
} ChasisBoardSend_101;
 /*********************************************************************
 * 设备通信总数据结构（汇总）
 *********************************************************************/
typedef struct {
    ChasisBoardReceive_110 ReceiveData_110;  // 接收：110 帧数据
    ChasisBoardReceive_120 ReceiveData_120;  // 接收：120 帧数据
    ChasisBoardReceive_130 ReceiveData_130;  // 接收：130 帧数据
    ChasisBoardSend_101    SendData;         // 发送：101 帧数据
} ChasisBoardData;

/*********************************************************************
 * 模块状态枚举（注册/初始化/收发状态）
 *********************************************************************/
typedef enum {
    AboveBoard_Register_OK      = 0, // 注册成功
    AboveBoard_Register_ERROR   = 1, // 注册失败
    AboveBoard_Init_OK          = 2, // 初始化成功
    AboveBoard_Init_ERROR       = 3, // 初始化失败
    AboveBoard_Reveive_OK       = 4, // 接收成功
    AboveBoard_Receive_ERROR    = 5, // 接收失败
    AboveBoard_Transmit_OK      = 6, // 发送成功
    AboveBoard_Transmit_ERROR   = 7  // 发送失败
} AboveBoard_STATUS;

/*********************************************************************
 * 接收解析函数声明
 *  - parameter_1：通常为目标结构体指针或设备句柄（按你的实现）
 *  - Data       ：原始 CAN/串口帧数据指针
 *********************************************************************/
void AboveBoardReceive_110(void* parameter_1, uint8_t* Data);
void AboveBoardReceive_120(void* parameter_1, uint8_t* Data);
void AboveBoardReceive_130(void* parameter_1, uint8_t* Data);
//发送函数声名
void AboveBoardSend_101(uint16_t Heat_Limit_Remain,uint16_t Heat_Limit_Recover,uint8_t Game_Programer,uint8_t Robot_ID,float Bullet_velocity);
