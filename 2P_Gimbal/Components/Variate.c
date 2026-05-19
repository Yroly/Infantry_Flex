#include "Variate.h"
int StuckFlag = 0;
uint8_t GimbalInitFlag = 0;

Gimbal_data_t Gimbal_data;
Gimbal_action_t Gimbal_action;
Chassis_RefereeMsg_t Referee_data_Rx;
Chassis_Msg_t Chassis_data_Rx;

uint8_t NormalModeFlag = 0,GyroscopeModeFlag = 0;
eSystemState SystemState;
DeviceStates DeviceState;
eMidMode MidMode = FRONT;
