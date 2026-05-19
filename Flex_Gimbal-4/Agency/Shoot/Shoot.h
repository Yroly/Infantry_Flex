#ifndef __SHOOT_H
#define __SHOOT_H

#include "Variate.h"
#include "USB_Task.h"

/* 决定控制方式 */
extern void ShootCtrl_Decide();
extern void Shoot_Rc_Ctrl();    //!< @brief 发射机构遥控器模式
extern void Shoot_Key_Ctrl();   //!< @brief 发射机构键鼠模式
extern void Shoot_Close();
/* 检测发射机构 */
extern void Detect_Shoot();
/* 设置目标量 */
extern void ShootRef_Set();
/* 枪口热量限制 */
extern void ShootHeat_Limit();
/* 计算控制量 */
extern void Shoot_Console();
extern void ShootSingle_Mode();
/* 发送控制量 */
extern void Shoot_Send();
extern void Shoot_SendDown();
#endif
