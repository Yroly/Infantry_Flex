#include "UpDown_Com.h"
#include "fdcan.h"
#include "bsp_can.h"

void AboveBoardReceive_110(void* parameter_1, uint8_t* Data)
{
    // 将 void 指针转换为 110 帧接收结构体指针
    ChasisBoardReceive_110 *parameter_2 = (ChasisBoardReceive_110*)parameter_1;

    /*
     * 解析接收到的 CAN 数据：
     * 采用小端格式：
     *   高字节在后（Data[1]），低字节在前（Data[0]）
     *
     * 示例：
     *   vx = 0x1234
     *   Data[0] = 0x34
     *   Data[1] = 0x12
     *   (0x12 << 8) | 0x34 = 0x1234
     */
    parameter_2->V_X = (Data[1] << 8) | Data[0];
    parameter_2->V_Y = (Data[3] << 8) | Data[2];
    parameter_2->Rotate = (Data[5] << 8) | Data[4];

    // 底盘关闭标志
    parameter_2->Close_flag = Data[6];

    // Shift 按键标志
    parameter_2->Shift_flag = Data[7];
}

void AboveBoardReceive_120(void* parameter_1, uint8_t* Data)
{
    // 将 void 指针转换为 120 帧接收结构体指针
    ChasisBoardReceive_120 *parameter_2 = (ChasisBoardReceive_120*)parameter_1;

    /*
     * Data[0]：
     *   bit0 -> Pitch 轴在线状态
     *   bit1 -> Yaw   轴在线状态
     */
    parameter_2->Gimbal_status.Pitch =
        (Gimbal_status_e)(Data[0] & 0x01);

    parameter_2->Gimbal_status.Yaw =
        (Gimbal_status_e)((Data[0] & 0x02) >> 1);

    /*
     * Data[1]：
     *   bit0     -> 射击系统状态
     *   bit1~4   -> 运动模式
     *   bit5     -> 视觉系统状态
     */
    parameter_2->shoot_status =
        (shoot_status_e)(Data[1] & 0x01);

    parameter_2->move_status =
        (move_status_e)((Data[1] & 0x1E) >> 1);

    parameter_2->vision_status =
        (vision_status_e)((Data[1] & 0x20) >> 5);

    // Data[2]：射击模式
    parameter_2->shoot_mode = (shoot_mode_e)Data[2];

    // Data[3]：按键状态汇总
    parameter_2->Key = Data[3];

    // Data[4]：视觉目标编号
    parameter_2->vision_number = Data[4];
}

/*
 * CAN 接收回调解析函数（130 帧）
 * 用于解析来自上位机 / 视觉系统的数据
 */
void AboveBoardReceive_130(void* parameter_1, uint8_t* Data)
{
    // 将 void 指针转换为 130 帧接收结构体指针
    ChasisBoardReceive_130 *parameter_2 = (ChasisBoardReceive_130*)parameter_1;

    /*
     * 解析接收到的 CAN 数据：
     * 采用大端顺序：
     *   高字节在前（Data[0]），低字节在后（Data[1]）
     */
    parameter_2->vision_distance = (Data[0] << 8) | Data[1];
    parameter_2->Pitch_angle     = (Data[2] << 8) | Data[3];
    parameter_2->Yaw_angle       = (Data[4] << 8) | Data[5];
    parameter_2->Offset_Angle    = (Data[6] << 8) | Data[7];
}

void AboveBoardSend_101(uint16_t Heat_Limit_Remain,uint16_t Heat_Limit_Recover,uint8_t Game_Programer,uint8_t Robot_ID,float Bullet_velocity){
	uint8_t Send101Buffer[8];
	uint16_t Send_ID_101=0x101;
    // 解析 Heat_Limit_Remain (uint16_t) -> 大端模式
    Send101Buffer[0] = (uint8_t)(Heat_Limit_Remain >> 8);   // 高 8 位
    Send101Buffer[1] = (uint8_t)(Heat_Limit_Remain & 0xFF); // 低 8 位

    // 解析 Heat_Limit_Recover (uint16_t) -> 大端模式
    Send101Buffer[2] = (uint8_t)(Heat_Limit_Recover >> 8);
    Send101Buffer[3] = (uint8_t)(Heat_Limit_Recover & 0xFF);

    // 填充单字节数据
    Send101Buffer[4] = Game_Programer;
    Send101Buffer[5] = Robot_ID;

    // 保留位填充（通常补0，防止随机数据干扰）
	
	uint16_t Bullet_velocity_u16=Bullet_velocity*100;
    Send101Buffer[6] =(uint8_t)(Bullet_velocity_u16 >> 8);
    Send101Buffer[7] =(uint8_t)(Bullet_velocity_u16 & 0xFF);
	canx_send_data(&hfdcan3, 0x101, Send101Buffer, 8);	
}
