#ifndef USB_TASK_H
#define USB_TASK_H

#include "Variate.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"

#define SEND_SOF    ((uint8_t)0x5A)
#define SEND_EOF    ((uint8_t)0xA5)
#define RECEIVE_SOF ((uint8_t)0x5A)
#define RECEIVE_EOF ((uint8_t)0xA5)

#define SEND_IMU_DATA_ID ((uint8_t)0x01)
#define RECEIVE_VISION_DATA_ID ((uint8_t)0x02)
#define SEND_TALLY_DATA_ID ((uint8_t)0x03)
/*------------- Header of Usb -------------*/
typedef struct{
	uint8_t sof;
	uint8_t len;
	uint8_t id;
} __PACKED FrameHeader_t;
/*------------- Ammor -------------*/
typedef struct{
  FrameHeader_t header;
  uint32_t time_stamp;
  struct{
		//自身颜色 0: blud  1: red		
		uint8_t self_color;
		//弹速 m/s
		float bullet_speed;
		//rad
    float yaw;
		float pitch;
		float roll;
		//rad/s²
		float yaw_vel;
		float pitch_vel;
		float roll_vel;
    } __PACKED data;
    uint8_t eof;
} __PACKED SendDataImu_s;

typedef struct{
  FrameHeader_t header;//3
  struct{
	  uint8_t FireFlag;
		//期望角度
		float Ref_Yaw;
		float Ref_Pitch;
		//速度前馈
		float Ref_Vyaw;
		float Ref_Vpitch;
		//角速度前馈
		float Ref_aYaw;
		float Ref_aPitch;
		//距离
		float dis;
  } __PACKED data;
  uint8_t eof;
} __PACKED ReceiveVisionData_t;
extern ReceiveVisionData_t ReceiveVisionData;
extern ReceiveVisionData_t last_ReceiveVisionData;
/*------------- Tally -------------*/
typedef struct{
	FrameHeader_t header;
	uint32_t time_stamp;
	struct{
		// 0: 未开启跟随 1: 开启跟随
		uint8_t following;
		// 0: 未开启开符模式 1: 开启开符模式
		uint8_t power_rune;
		// 0: 未开启 Quanta 图传 1: 开启 Quanta 图传
		uint8_t quanta;
	} __PACKED data;
	uint8_t eof; // 0xA5
}__PACKED SendTallyData_t;
/*------------- Quanta -------------*/
typedef struct{
	FrameHeader_t header;
	uint32_t time_stamp;
	struct{
		// 0: 未开启跟随 1: 开启跟随
		uint8_t following;
		// 0: 未开启开符模式 1: 开启开符模式
		uint8_t power_rune;
		// 0: 未开启 Quanta 图传 1: 开启 Quanta 图传
		uint8_t quanta;
	} __PACKED data;
	uint8_t eof; // 0xA5
}__PACKED SendQuantaData_t;
extern void usb_task();

#endif /* USB_TASK_H */
