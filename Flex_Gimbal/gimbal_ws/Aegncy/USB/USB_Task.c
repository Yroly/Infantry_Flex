#include "USB_Task.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

static SendImuData_t SendImuData = {.header.sof = 0x5A,
																		.header.len = (uint8_t)(sizeof(SendImuData_t) - 6),
																		.header.id  = 0x01,
																		.eof = 0xA5};
ReceiveVisionData_t ReceiveVisionData = {.header.sof = 0x5A,
																				 .header.id  = 0X02,
																				 .eof = 0xA5,
																				 .data.dis = -1};
static void UsbInit(void);
static void UsbReceiveData(void);
static void UsbSendImuData(void);

void usb_task(void *pvParameters){
	UsbInit();
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;){
			UsbSendImuData();
			UsbReceiveData();
			vTaskDelayUntil(&xLastWakeTime,1);
	}
}
static void UsbInit(void){
	memset(&SendImuData.data,0,sizeof(SendImuData.data));
	memset(&ReceiveVisionData.data,0,sizeof(ReceiveVisionData.data));
}

static void UsbReceiveData(void) {
	static uint8_t data_buffer[64] = {0};
  uint32_t actual_len = 0;
//  USB_Receive(data_buffer, &actual_len);
	memcpy(&ReceiveVisionData, data_buffer, sizeof(ReceiveVisionData_t));
}

static void UsbSendImuData(void){
//	SendImuData.time_stamp = HAL_GetTick();//获取当前时间戳
//	SendImuData.data.bullet_speed = SHOOT_SPEED * 0.10472f * 0.03;//m/s 733.04是rpm转化为线速度 v=wr;
//	SendImuData.data.pitch = 	IMU.Angle_Pitch *Pi/180.0f;//rad
//	SendImuData.data.yaw 	 = 	IMU.Angle_Yaw 	*Pi/180.0f;
//	SendImuData.data.roll  =	IMU.Angle_Roll 	*Pi/180.0f;
//	SendImuData.data.pitch_vel = IMU.Gyro_Pitch;
//	SendImuData.data.yaw_vel 	 = IMU.Gyro_Yaw;  
//	SendImuData.data.roll_vel  = IMU.Gyro_Roll; 
//	SendImuData.data.self_color = 1 ;//1蓝0红

//  USB_Transmit((uint8_t *)&SendImuData, sizeof(SendImuData_t));
}
