#pragma once

#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "usbd_conf.h"

typedef struct{
	uint8_t sof;
	uint8_t len;
	uint8_t id;
}__PACKED FrameHeader_t;
/*-------------------- Send --------------------*/
typedef struct{
	FrameHeader_t header;
	uint32_t time_stamp;
	struct{
		uint8_t self_color ; //0-blud  1-red
		float bullet_speed;// m/s
		float yaw;    // rad
		float pitch;  // rad
		float roll;   // rad

		float yaw_vel;    // rad/s
		float pitch_vel;  // rad/s
		float roll_vel;   // rad/s
	}__PACKED data;
	uint8_t eof;
}__PACKED SendImuData_t;
/*-------------------- Receive --------------------*/
typedef struct{
  FrameHeader_t header;//3
  struct{
	  uint8_t FireFlag;
		float Ref_Yaw;//°
		float Ref_Pitch;
		float Ref_Vyaw;
		float Ref_Vpitch;
		float Ref_aYaw;
		float Ref_aPitch;
		float dis;
  } __PACKED data;
  uint8_t eof;//1
} __PACKED ReceiveVisionData_t;
extern ReceiveVisionData_t ReceiveVisionData;

extern void Usb_Task();
