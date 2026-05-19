#ifndef _CANDrive_H_
#define _CANDrive_H_

#include "can.h"
extern CAN_RxHeaderTypeDef RxHeader[2];
extern uint8_t CAN1_buff[8];        //!<@brief CAN1接收缓冲区
extern uint8_t CAN2_Rxbuff[64];        //!<@brief CAN2接收缓冲区
/**
 * @brief 按照通常设置初始化CAN滤波器
 * @param hcan CAN handle Structure definition
 */
void CanFilter_Init(CAN_HandleTypeDef* hcan);
/**
 * @brief CAN发送标准帧数据
 * @param hcan CAN句柄
 * @param[in] StdId 标准帧ID
 * @param[in] msg 数据数组,长度为8
 * @return HAL Status structures definition
 */
HAL_StatusTypeDef CAN_Send_StdDataFrame(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *msg);

/**
 * @brief CAN读取数据
 * @param hcan CAN句柄
 * @param[out] buf 数据缓冲区
 * @return 标准帧ID或拓展帧ID
 */
uint32_t CAN_Receive_DataFrame(CAN_HandleTypeDef *hcan, uint8_t *buf);

#endif //_CANDrive_H_
