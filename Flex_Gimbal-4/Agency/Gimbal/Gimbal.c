#include "Gimbal.h"
#include "Time.h"
#include "USB_Task.h"
#include "dm_motor.h"
#include "VT03.h"
 
eGimbal Gimbal;
eGimbalCtrl GimbalCtrl;
KFF_t KFF = {.Yaw = 0.0f,.aPitch = 1.0f,.aYaw = 1.0f,.vPitch = 1.0f,.vYaw = 1.0f};
int16_t Can2Send[4] = {0};

PID_TypeDef Gimbal_Speed_pid_Yaw[GIMBAL_MODE];
PID_TypeDef Gimbal_Place_pid_Yaw[GIMBAL_MODE];

PID_TypeDef Gimbal_Place_pid_Pitch[GIMBAL_MODE];
PID_TypeDef Gimbal_Speed_pid_Pitch[GIMBAL_MODE];
/* 云台初始化 */  
void GimbalInit(){
	GimbalCtrl = gNormal;
	GimbalInitFlag = 1;
	Time.GimbalInit = 0;
  
	PID_init(&Gimbal_Place_pid_Yaw[INIT],16384,0,0,2.0,0,0,0,0);	
	PID_init(&Gimbal_Speed_pid_Yaw[INIT],16384,0,0,3.0,0,0,0,0);	
	PID_init(&Gimbal_Place_pid_Yaw[GYRO],16384,1,0,50,0,200,0,0);	
	PID_init(&Gimbal_Speed_pid_Yaw[GYRO],16384,1,0,5,0,0,0,0);	
	PID_init(&Gimbal_Place_pid_Yaw[AIM],16384,0,0,50,0,200,0,0);	
	PID_init(&Gimbal_Speed_pid_Yaw[AIM],16384,0,0,5,0,0,0,0);	

	PID_init(&Gimbal_Place_pid_Pitch[INIT],16384,0,0,0.0,0,0,0,0);	
	PID_init(&Gimbal_Speed_pid_Pitch[INIT],16384,0,0,0.0,0,0,0,0);	
	PID_init(&Gimbal_Place_pid_Pitch[GYRO],16384,1,0,50,0,100,0,0);	
	PID_init(&Gimbal_Speed_pid_Pitch[GYRO],16384,1,0,-10.0,0,0,0,0);	
	PID_init(&Gimbal_Place_pid_Pitch[AIM],16384,0,0,50,0,100,0,0);	
	PID_init(&Gimbal_Speed_pid_Pitch[AIM],16384,0,0,-10,0,0,0,0);	
}
/*控制方式决定函数*/
void GimbalCtrl_Decide(){
	if(DeviceState.VT03_State == Device_Online){
		VT03.mode == ComInput ? Gimbal_Key_Ctrl() :
		VT03.mode == RcInput ? Gimbal_RC_Ctrl() :
		Gimbal_Stop();
	} else Gimbal_Stop();
}
/* 遥控器控制 */
void Gimbal_RC_Ctrl(){
	static char Rc_FNl_flag = 0;
  if(GimbalCtrl == gStop) GimbalCtrl = gNormal;
	if(VT03.fn_l == 1 && Rc_FNl_flag == 0 ){
		if(GimbalCtrl != gAim) GimbalCtrl = gAim;
		else GimbalCtrl = gNormal;
		Rc_FNl_flag = 1;
	}
	if(VT03.fn_l == 0) Rc_FNl_flag = 0;	
}
/* 键鼠控制 */
void Gimbal_Key_Ctrl(){
	static char Key_F_flag = 0 , mouse_r_flag = 0;
  if(GimbalCtrl == gStop) GimbalCtrl = gNormal;
  /* 普通模式F快速转头 */
	if(GimbalCtrl != gAim){
		if(VT03.keys.F == 1 && Key_F_flag == 0){
			Gimbal.Ref[YAW] += 180;
			Key_F_flag = 1;
		}
		if(VT03.keys.F == 0) Key_F_flag = 0;
	}
  /* 鼠标右键自瞄模式*/
	if(VT03.mouse.right == 1 && mouse_r_flag == 0 ){
		if(GimbalCtrl != gAim) GimbalCtrl = gAim;
		else GimbalCtrl = gNormal;
		mouse_r_flag = 1;
	}
	if(VT03.mouse.right== 0) mouse_r_flag = 0;
}
/**
 *@brief 云台急停
 */
void Gimbal_Stop(){
  GimbalCtrl = gStop;
}
/**
 *@brief 云台期望值更新
 */
void GimbalRef_Update(){
  float yaw_diff = 0;
  yaw_diff = ReceiveVisionData.data.Ref_Yaw - IMU.Angle_Yaw;
  if(yaw_diff > 180.0f){
    yaw_diff -= 360.0f;
  } else if (yaw_diff < -180.0f){
    yaw_diff += 360.0f;
  }
  IMU.VisionAngle = IMU.Angle_Yawcontinuous + yaw_diff;

  switch(GimbalCtrl){
	case gNormal:
		if(VT03.mode == ComInput){
			Gimbal.increase[PITCH]  = VT03.mouse.vy * 0.1f;
			Gimbal.increase[YAW]    = VT03.mouse.vx * 0.1f;
		} else if (VT03.mode == RcInput){
			Gimbal.increase[PITCH] = -VT03.ch_ly * 0.1f;
			Gimbal.increase[YAW]   = VT03.ch_lx * 0.2f;
		}                                                
		Gimbal.Ref[PITCH] -= Gimbal.increase[PITCH];
		Gimbal.Ref[YAW] -= Gimbal.increase[YAW];			
		limit(Gimbal.Ref[PITCH],P_ADD_limit,P_LOSE_limit);		 
		break;
	case gAim:
		if(DeviceState.PC_State == 1 && ReceiveVisionData.data.dis > 0.1f){
			Gimbal.increase[YAW]   = 0;
			Gimbal.increase[PITCH] = 0;
			Gimbal.Ref[YAW] = IMU.VisionAngle;
			Gimbal.Ref[PITCH] = ReceiveVisionData.data.Ref_Pitch;
      limit(Gimbal.Ref[PITCH] ,P_ADD_limit,P_LOSE_limit);
		} else {
			if(VT03.mode == ComInput){
				Gimbal.increase[YAW]   = VT03.mouse.vx * 0.25;
				Gimbal.increase[PITCH] = VT03.mouse.vy * 0.01;
			} else if (VT03.mode == RcInput){				
				Gimbal.increase[YAW]   = VT03.ch_lx * 0.3f;
				Gimbal.increase[PITCH] = -VT03.ch_ly * 0.01f;
			}
			Gimbal.Ref[PITCH] -= Gimbal.increase[PITCH];
			Gimbal.Ref[YAW] -= Gimbal.increase[YAW];
		}
		Gimbal.LastCtrl = gAim;
		break;
  case gStop : 
      Gimbal.increase[YAW]   = 0;
      Gimbal.increase[PITCH] = 0;
      Gimbal.Ref[PITCH] = IMU.Angle_Pitch;
      Gimbal.Ref[YAW]   = IMU.Angle_Yawcontinuous;    
    break;
  }
}
float FF_Yaw = 0.0f;
void Gimbal_Calc(){
	FF_Yaw = Chassis_data_Rx.Chassis_Speed * KFF.Yaw;
	if(GimbalCtrl == gAim && DeviceState.PC_State == 1 && ReceiveVisionData.data.dis > 0.1f){
		PID_Calc(&Gimbal_Place_pid_Pitch[AIM],IMU.Angle_Pitch,Gimbal.Ref[PITCH]);
		PID_Calc(&Gimbal_Speed_pid_Pitch[AIM],IMU.Gyro_Pitch,Gimbal_Place_pid_Pitch[AIM].Output + ReceiveVisionData.data.Ref_Vpitch * KFF.vPitch);		
				
		PID_Calc(&Gimbal_Place_pid_Yaw[AIM],IMU.Angle_Yawcontinuous,Gimbal.Ref[YAW]);
		PID_Calc(&Gimbal_Speed_pid_Yaw[AIM],IMU.Gyro_Yaw,Gimbal_Place_pid_Yaw[AIM].Output + ReceiveVisionData.data.Ref_Vyaw * KFF.vYaw + FF_Yaw);		
	} else {
		PID_Calc(&Gimbal_Place_pid_Pitch[GYRO],IMU.Angle_Pitch,Gimbal.Ref[PITCH]);
		PID_Calc(&Gimbal_Speed_pid_Pitch[GYRO],IMU.Gyro_Pitch,Gimbal_Place_pid_Pitch[GYRO].Output);		
		
		PID_Calc(&Gimbal_Place_pid_Yaw[GYRO],IMU.Angle_Yawcontinuous,Gimbal.Ref[YAW]);
		PID_Calc(&Gimbal_Speed_pid_Yaw[GYRO],IMU.Gyro_Yaw,Gimbal_Place_pid_Yaw[GYRO].Output);		
	}
}
void Gimbal_Send(){
	if(GimbalCtrl == gAim && DeviceState.PC_State == 1 && ReceiveVisionData.data.dis > 0.1f){
		Can2Send[2] = (int16_t)(Gimbal_Speed_pid_Pitch[AIM].Output + ReceiveVisionData.data.Ref_aPitch * KFF.aPitch);	
		Can2Send[3] = (int16_t)(Gimbal_Speed_pid_Yaw[AIM].Output + ReceiveVisionData.data.Ref_aYaw * KFF.aYaw);	
	} else if(GimbalCtrl == gStop){
    Can2Send[2] = Can2Send[3] = 0;
	} else {
		Can2Send[2] = (int16_t)(Gimbal_Speed_pid_Pitch[GYRO].Output);	
		Can2Send[3] = (int16_t)(Gimbal_Speed_pid_Yaw[GYRO].Output);	
  }
#if GIMBAL_RUN
	DM_MotorSend(&hcan2,0x3FE,Can2Send);
#endif
	if(DeviceState.Gimbal_State[PITCH] == Device_Online) Gimbal_action.Gimbal_status.Pitch = Gimbal_online;
		else Gimbal_action.Gimbal_status.Pitch 	= 	Gimbal_offline;
	if( DeviceState.Gimbal_State[YAW]  == Device_Online)  Gimbal_action.Gimbal_status.Yaw  = Gimbal_online;
		else Gimbal_action.Gimbal_status.Yaw 	= 	Gimbal_offline; 
}
/* 云台归中函数 */
void MedianInit(){
	static float Expect_PitchInit = 0;
  static float Expect_YawInit = 0;
  uint16_t Expect_YawRamp   = GimYaw.MchanicalAngle;
  uint16_t Expect_PitchRamp = GimPitch.MchanicalAngle;
	/* 获得归中位置 */
	if(Time.GimbalInit < 50){
#if   Yaw_Mid_Right < Yaw_Mid_Left
        if ( (GimYaw.MchanicalAngle <= Yaw_Mid_Left) && (GimYaw.MchanicalAngle >= Yaw_Mid_Right) )
#elif Yaw_Mid_Right > Yaw_Mid_Left
        if ( (GimYaw.MchanicalAngle <= Yaw_Mid_Left) || (GimYaw.MchanicalAngle >= Yaw_Mid_Right) )
#endif
		MidMode = FRONT; else MidMode = BACK;
	} else {
		if (MidMode == FRONT) Expect_YawInit = QuickCentering(GimYaw.MchanicalAngle,Yaw_Mid_Front);
		else Expect_YawInit = QuickCentering(GimYaw.MchanicalAngle,Yaw_Mid_Back);
	}
	/* Yaw come mid */
	Expect_YawRamp = RAMP_float(Expect_YawInit,Expect_YawRamp,200); 
	PID_Calc(&Gimbal_Place_pid_Yaw[INIT],GimYaw.MchanicalAngle,Expect_YawRamp);
	PID_Calc(&Gimbal_Speed_pid_Yaw[INIT],GimYaw.Speed,Gimbal_Place_pid_Yaw[INIT].Output);
  limit(Gimbal_Speed_pid_Yaw[INIT].Output,GM6020_LIMIT,-GM6020_LIMIT);
	/* Pitch come mid*/	
	Expect_PitchInit = QuickCentering(GimPitch.MchanicalAngle,Pitch_Mid);
	Expect_PitchRamp = RAMP_float(Expect_PitchInit,Expect_PitchRamp,50); 
	PID_Calc(&Gimbal_Place_pid_Pitch[INIT],GimPitch.MchanicalAngle,Expect_PitchRamp);
	PID_Calc(&Gimbal_Speed_pid_Pitch[INIT],GimPitch.Speed,Gimbal_Place_pid_Pitch[INIT].Output);
	limit( Gimbal_Speed_pid_Pitch[INIT].Output,GM6020_LIMIT,-GM6020_LIMIT );

	Can2Send[2] = (int16_t)Gimbal_Place_pid_Pitch[INIT].Output;
	Can2Send[3] = (int16_t)Gimbal_Speed_pid_Yaw[INIT].Output;
	limit(Can2Send[GIMBAL_SUM],GM6020_LIMIT,-GM6020_LIMIT);	
#if GIMBAL_RUN
	DM_MotorSend(&hcan2,0x3FE,Can2Send);
#endif	
	if(Time.GimbalInit >= 1000){
		Time.GimbalInit = 0;
		GimbalInitFlag  = 0;
		Gimbal.YawInit   = Expect_YawInit;
		Gimbal.PitchInit = Expect_PitchInit;

		Gimbal.increase[PITCH] = 0;
		Gimbal.increase[YAW]   = 0;
		Gimbal.Ref[YAW] = IMU.Angle_Yawcontinuous;
		Gimbal.Ref[PITCH] = IMU.Angle_Pitch;
		
		SystemState = SYSTEM_RUNNING;
	}
}
/* CAN2回调接收 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
  if (hcan->Instance == CAN2) {
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&RxHeader[1],CAN2_Rxbuff);
    switch (RxHeader[1].StdId){
			case 0x101:
						Referee_data_Rx.game_state = CAN2_Rxbuff[0];
						Referee_data_Rx.robot_color = CAN2_Rxbuff[1];
						Referee_data_Rx.heat_limit = (uint16_t)(CAN2_Rxbuff[2] << 8 | CAN2_Rxbuff[3]);
						Referee_data_Rx.heat_cooling = (uint16_t)(CAN2_Rxbuff[4] << 8 | CAN2_Rxbuff[5]);
						Referee_data_Rx.heat_now = (uint16_t)(CAN2_Rxbuff[6] << 8 | CAN2_Rxbuff[7]);
                    Feed_Dog(&Referee_Dog);
				break;
			case 0x102:
						memcpy(&Chassis_data_Rx.Chassis_Speed,&CAN2_Rxbuff[0],sizeof(float));
						memcpy(&Chassis_data_Rx.bullet_speed,&CAN2_Rxbuff[4],sizeof(float));
										Feed_Dog(&Down_Dog);
				break;
			case 0x303 : DM4310_Receive(&GimPitch,CAN2_Rxbuff);
					Feed_Dog(&Gimbal_Dog[PITCH]);
				break;
			case 0x304 : DM4310_Receive(&GimYaw,CAN2_Rxbuff);
					Feed_Dog(&Gimbal_Dog[YAW]);
				break;
      default:	break;
    }
  }
}
