#pragma once

#include "stdint.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

#define WatchDoglength 10
#define IS_Dog(handle, Dog) ((handle) == &(Dog))

typedef enum{
	SYSTEM_STARTING = 0,
	SYSTEM_RUNNING	= 1,
}SystemState_e;
typedef enum{
	Device_Offline = 0,
	Device_Online  = 1,
	Device_Error   = 2
}DeviceState_e;
typedef struct{
	DeviceState_e State;
	uint32_t Life;  //!<@brief 当前离线计数
	uint32_t Max;   //!<@brief 最大离线计数
 }WatchDog_TypeDef,*WatchDogP;

class WatchDog
{
public:
	WatchDog_TypeDef Remote_Dog;
	WatchDog_TypeDef Chassis_Dog[4];
	WatchDog_TypeDef Joint_Dog[2];

  void init(WatchDogP handle,uint32_t life);
	void WatchBack(WatchDogP handle);
  void FeedBack(WatchDogP handle);
	void polling();
  void feed(WatchDogP handle);
};
extern WatchDog dog;
