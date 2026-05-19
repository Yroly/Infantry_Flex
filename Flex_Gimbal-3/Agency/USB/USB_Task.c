#include "usb_task.h"
#include "Gimbal.h"
static SendDataImu_s SEND_DATA_IMU = {.header.sof = 0x5A,
																			.header.len = (uint8_t)(sizeof(SendDataImu_s) - 6),
																			.header.id  = 0x01,
																			.eof = 0xA5};
ReceiveVisionData_t ReceiveVisionData = {.header.sof = 0x5A,
																				 .header.id  = 0X02,
																				 .eof = 0xA5,
																				 .data.dis = -1};
static SendTallyData_t SendTallyData = {.header.sof = 0x5A,
																				.header.len = (uint8_t)(sizeof(SendTallyData_t) - 6),
																				.header.id = 0x03,
																				.eof = 0xA5};


static void UsbInit(void);
static void UsbReceiveData(void);
static void UsbSendImuData(void);
static void UsbSendTallyData(void);
																				 
void usb_task(void *pvParameters){
	UsbInit();
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;){
		UsbSendImuData();
		UsbSendTallyData();
		UsbReceiveData();
		vTaskDelayUntil(&xLastWakeTime,1);
	}
}
static void UsbInit(void){
	memset(&SEND_DATA_IMU.data,0,sizeof(SEND_DATA_IMU.data));
	memset(&SendTallyData.data,0,sizeof(SendTallyData.data));
	memset(&ReceiveVisionData.data,0,sizeof(ReceiveVisionData.data));
}
static void UsbSendTallyData(void){
	SendTallyData.time_stamp = HAL_GetTick();//获取当前时间戳
	
	if(GimbalCtrl == gAim) SendTallyData.data.following = 1;
	else SendTallyData.data.following = 0;
	if(VT03.keys.B == 1){
		SendTallyData.data.power_rune = 1;	
	}
	SendTallyData.data.quanta = 0;
  USB_Transmit((uint8_t *)&SendTallyData, sizeof(SendTallyData_t));	
}
static void UsbSendImuData(void){
	SEND_DATA_IMU.time_stamp = HAL_GetTick();//获取当前时间戳
	SEND_DATA_IMU.data.bullet_speed = Chassis_data_Rx.bullet_speed;
	SEND_DATA_IMU.data.pitch = 	IMU.Angle_Pitch * PI / 180.0f;//rad
	SEND_DATA_IMU.data.yaw 	 = 	IMU.Angle_Yaw * PI / 180.0f;
	SEND_DATA_IMU.data.roll  =	IMU.Angle_Roll * PI / 180.0f;
	SEND_DATA_IMU.data.pitch_vel = IMU.Gyro_Pitch * PI / 180.0f;
	SEND_DATA_IMU.data.yaw_vel 	 = IMU.Gyro_Yaw * PI / 180.0f;  
	SEND_DATA_IMU.data.roll_vel  = IMU.Gyro_Roll * PI / 180.0f; 
	SEND_DATA_IMU.data.self_color = Referee_data_Rx.robot_color;
  USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}
static void UsbReceiveData(void){
	static uint8_t data_buffer[64] = {0};
  uint32_t actual_len = 0;
  USB_Receive(data_buffer, &actual_len);
	ReceiveVisionData_t last_ReceiveVisionData = ReceiveVisionData;
	memcpy(&ReceiveVisionData, data_buffer, sizeof(ReceiveVisionData_t));
	if(memcmp(&ReceiveVisionData,&last_ReceiveVisionData,sizeof(ReceiveVisionData_t))!=0){
		Feed_Dog(&PC_Dog);	
	}
}
