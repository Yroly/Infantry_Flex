#include "Gimbal.h"
#include "Time.h"
#include "USB_Task.h"
#include "dm_motor.h"
#include "VT03.h"

eGimbal Gimbal;
eGimbalCtrl GimbalCtrl;
int16_t Can2Send[4] = {0};
uint8_t goal_flag = 0;
PID_TypeDef Gimbal_Place_pid_UnderP[GIMBAL_MODE];
PID_TypeDef Gimbal_Speed_pid_UnderP[GIMBAL_MODE];

PID_TypeDef Gimbal_Place_pid_Pitch[GIMBAL_MODE];
PID_TypeDef Gimbal_Speed_pid_Pitch[GIMBAL_MODE];

PID_TypeDef Gimbal_Place_pid_Yaw[GIMBAL_MODE];
PID_TypeDef Gimbal_Speed_pid_Yaw[GIMBAL_MODE];


void GimbalInit(){
	GimbalCtrl = gNormal;
	GimbalInitFlag = 1;
	Time.GimbalInit = 0;
	
	PID_init(&Gimbal_Place_pid_UnderP[INIT],16384,0,0,2.0,0,0,0,0);	
	PID_init(&Gimbal_Speed_pid_UnderP[INIT],16384,0,0,5.0,0,0,0,0);	
	PID_init(&Gimbal_Place_pid_UnderP[GYRO],16384,1,0,20,0,0,0,0);	 
	PID_init(&Gimbal_Speed_pid_UnderP[GYRO],16384,1,0,2,0,0,0,0);	
	PID_init(&Gimbal_Place_pid_UnderP[AIM],16384,0,0,0,0,0,0,0);	
	PID_init(&Gimbal_Speed_pid_UnderP[AIM],16384,0,0,0,0,0,0,0);
	
	PID_init(&Gimbal_Place_pid_Pitch[INIT],16384,0,0,5,0,0,0,0);	
	PID_init(&Gimbal_Speed_pid_Pitch[INIT],16384,0,0,3,0,0,0,0);	
	PID_init(&Gimbal_Place_pid_Pitch[GYRO],16384,1,0,10,0,100,0,0);	
	PID_init(&Gimbal_Speed_pid_Pitch[GYRO],16384,1,0,-50,0,0,0,0);	
	PID_init(&Gimbal_Place_pid_Pitch[AIM],16384,0,0,0,0,0,0,0);	
	PID_init(&Gimbal_Speed_pid_Pitch[AIM],16384,0,0,0,0,0,0,0);

	PID_init(&Gimbal_Place_pid_Yaw[INIT],16384,0,0,2,0,0,0,0);	
	PID_init(&Gimbal_Speed_pid_Yaw[INIT],16384,0,0,3,0,0,0,0);	
	PID_init(&Gimbal_Place_pid_Yaw[GYRO],16384,1,0,5.0f,0,200.0f,0,0);	
	PID_init(&Gimbal_Speed_pid_Yaw[GYRO],16384,1,0,60.0f,0.0,0,0,0);	
	PID_init(&Gimbal_Place_pid_Yaw[MECH],16384,1,0,5,0,0,0,0);	
	PID_init(&Gimbal_Speed_pid_Yaw[MECH],16384,1,0,50,0,0,0,0);	
	PID_init(&Gimbal_Place_pid_Yaw[AIM],16384,0,0,5.0,0,150.0,0,0);	
	PID_init(&Gimbal_Speed_pid_Yaw[AIM],16384,0,0,50.0f,0,0,0,0);	
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
	static char Rc_FNl_flag = 0,Fn_R_Flag = 0;
  if(GimbalCtrl == gStop) GimbalCtrl = gNormal;
	
	if(VT03.fn_l == 1 && Rc_FNl_flag == 0 ){
		if(GimbalCtrl != gAim) GimbalCtrl = gAim;
		else GimbalCtrl = gNormal;
		Rc_FNl_flag = 1;
	}
	if(VT03.fn_l == 0) Rc_FNl_flag = 0;
	
	if(VT03.fn_r == 1 && Fn_R_Flag == 0){
		if(goal_flag != 1) goal_flag = 1;
		else{
			goal_flag = 0;
			Gimbal.Ref[YAW] = IMU.Angle_Yawcontinuous;
		}		
	}
	if(VT03.fn_r == 0) Fn_R_Flag = 0;
}
/* 键鼠控制 */
void Gimbal_Key_Ctrl(){
  if(GimbalCtrl == gStop) GimbalCtrl = gNormal;
	static char Key_F_flag = 0, Key_Q_flag = 0, mouse_r_flag = 0;
	if(GimbalCtrl != gAim){
    /* F转头180 */
		if(VT03.keys.F == 1 && Key_F_flag == 0){
			Gimbal.Ref[YAW] += 180;
			Key_F_flag = 1;
		}
		if(VT03.keys.F == 0) Key_F_flag = 0;
	}
  /* Q按下过洞模式 */
  if(VT03.keys.Q == 1 && Key_Q_flag == 0){
    
    Key_Q_flag = 1;
  }
  if(VT03.keys.Q == 0) Key_Q_flag = 0;
  /* 右键云台自瞄模式控制 */
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
  /* 自瞄角度连续化处理 */
  float yaw_diff = 0;
  yaw_diff = ReceiveVisionData.data.Ref_Yaw - IMU.Angle_Yaw;
  if(yaw_diff > 180.0f){
    yaw_diff -= 360.0f;
  } else if (yaw_diff < -180.0f){
    yaw_diff += 360.0f;
  }
  IMU.VisionAngle = IMU.Angle_Yawcontinuous + yaw_diff;

	float goal_yaw_diff = 0;
	goal_yaw_diff = TUNNEL_MID_YAW - GimYaw.Angle_DEG;
  if(goal_yaw_diff > 180.0f){
    goal_yaw_diff -= 360.0f;
  } else if (goal_yaw_diff < -180.0f){
    goal_yaw_diff += 360.0f;
  }
	
  
  switch(GimbalCtrl){
    case gNormal:
			//进入过洞模式
			if(goal_flag == 1){
				if(VT03.mode == ComInput){
					Gimbal.Ref[YAW] = GimYaw.Angle_DEG + goal_yaw_diff;
					Gimbal.Ref[UnderP] = TO_GOAL_ANGLE;
					Gimbal.increase[PITCH]  = VT03.mouse.vy * 0.1f;					
				} else if (VT03.mode == RcInput){
					Gimbal.Ref[YAW] = GimYaw.Angle_DEG + goal_yaw_diff;
					Gimbal.Ref[UnderP] = TO_GOAL_ANGLE;
					Gimbal.increase[PITCH]  = VT03.ch_ly * 0.1f;										
				}	
				Gimbal.Ref[PITCH] -= Gimbal.increase[PITCH];
				limit(Gimbal.Ref[UnderP],UNP_ADD_limit,UNP_LOSE_limit);
				limit(Gimbal.Ref[PITCH],P_ADD_limit,P_LOSE_limit);		 								
			} else {
				//判断控制模式 Com键鼠模式 Rc遥控模式
				if(VT03.mode == ComInput){
					Gimbal.increase[YAW]    = VT03.mouse.vx * 0.1f;
					Gimbal.Ref[UnderP] = UNP_ADD_limit;
					Gimbal.increase[PITCH]  = VT03.mouse.vy * 0.1f;
				} else if (VT03.mode == RcInput){
					Gimbal.increase[YAW]   = VT03.ch_lx * 0.1f;
					Gimbal.Ref[UnderP] = UNP_ADD_limit;
					Gimbal.increase[PITCH] = VT03.ch_ly * 0.1f;
				}
				Gimbal.Ref[PITCH] -= Gimbal.increase[PITCH];
				Gimbal.Ref[YAW] -= Gimbal.increase[YAW];
				limit(Gimbal.Ref[UnderP],UNP_ADD_limit,UNP_LOSE_limit);
				limit(Gimbal.Ref[PITCH],P_ADD_limit,P_LOSE_limit);		 				
			}
      break;
    case gAim:
			//自瞄在线且识别到目标
      if(DeviceState.PC_State == 1 && ReceiveVisionData.data.dis > 0.1f){
        Gimbal.increase[YAW]   = 0;
        Gimbal.increase[PITCH] = 0;
        Gimbal.Ref[YAW] = IMU.VisionAngle;
        Gimbal.Ref[PITCH] = ReceiveVisionData.data.Ref_Pitch;
        limit(Gimbal.Ref[PITCH],P_ADD_limit,P_LOSE_limit);
      } else {
        if(VT03.mode == ComInput){
          Gimbal.increase[YAW]   = VT03.mouse.vx * 0.15;
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
				Gimbal.increase[UnderP] = 0;
				Gimbal.Ref[UnderP] = UnderPitch.Angle_DEG;
        Gimbal.Ref[PITCH] = IMU.Angle_Pitch;
        Gimbal.Ref[YAW]   = IMU.Angle_Yawcontinuous;    
      break;
   }
}
float FF_Yaw = 0.0f, K_FF = 0.0f;
void Gimbal_Calc(){
	static char yawgoal_flag = 0,underp_flag = 0;
	FF_Yaw = Chassis_data_Rx.Chassis_Speed * K_FF;
	if(GimbalCtrl == gAim && DeviceState.PC_State == 1 && ReceiveVisionData.data.dis > 0.1f){
		PID_Calc(&Gimbal_Place_pid_Pitch[AIM],IMU.Angle_Pitch,Gimbal.Ref[PITCH]);
		PID_Calc(&Gimbal_Speed_pid_Pitch[AIM],IMU.Gyro_Pitch,Gimbal_Place_pid_Pitch[AIM].Output + ReceiveVisionData.data.Ref_Vpitch);		
				
		PID_Calc(&Gimbal_Place_pid_Yaw[AIM],IMU.Angle_Yawcontinuous,Gimbal.Ref[YAW]);
		PID_Calc(&Gimbal_Speed_pid_Yaw[AIM],IMU.Gyro_Yaw,Gimbal_Place_pid_Yaw[AIM].Output + ReceiveVisionData.data.Ref_Vyaw + FF_Yaw);		
	} else {
		if(goal_flag == 1){
			PID_Calc(&Gimbal_Place_pid_Yaw[MECH],GimYaw.Angle_DEG,Gimbal.Ref[YAW]);
			PID_Calc(&Gimbal_Speed_pid_Yaw[MECH],GimYaw.Speed,Gimbal_Place_pid_Yaw[MECH].Output);
			if(Gimbal.Ref[YAW] - GimYaw.Angle_DEG < 2.0f){
				PID_Calc(&Gimbal_Place_pid_UnderP[GYRO],UnderPitch.Angle_DEG,Gimbal.Ref[UnderP]);
				PID_Calc(&Gimbal_Speed_pid_UnderP[GYRO],UnderPitch.Speed,Gimbal_Place_pid_UnderP[GYRO].Output);													
			}
		} else {
			PID_Calc(&Gimbal_Place_pid_Yaw[GYRO],IMU.Angle_Yawcontinuous,Gimbal.Ref[YAW]);
			PID_Calc(&Gimbal_Speed_pid_Yaw[GYRO],IMU.Gyro_Yaw,Gimbal_Place_pid_Yaw[GYRO].Output);				

			PID_Calc(&Gimbal_Place_pid_UnderP[GYRO],UnderPitch.Angle_DEG,Gimbal.Ref[UnderP]);
			PID_Calc(&Gimbal_Speed_pid_UnderP[GYRO],UnderPitch.Speed,Gimbal_Place_pid_UnderP[GYRO].Output);							
		}
			PID_Calc(&Gimbal_Place_pid_Pitch[GYRO],IMU.Angle_Pitch,Gimbal.Ref[PITCH]);
			PID_Calc(&Gimbal_Speed_pid_Pitch[GYRO],IMU.Gyro_Pitch,Gimbal_Place_pid_Pitch[GYRO].Output);		
	}
}
float GimbalPitchOffset = 0;
float KPitch = 1000.0f;
void Gimbal_Send(){
	if(IMU.Angle_Pitch < 15.0f && IMU.Angle_Pitch > -35.0f)
		    GimbalPitchOffset = KPitch * cos(fabs(IMU.Angle_Pitch)*PI/180.0f);
//		else if(ins->Pitch >= 20.0f)
//			GimbalPitchOffset = -12000.0f * sin(fabs(ins->Pitch)*PI/180.0f);
	else GimbalPitchOffset = 0;
	
	if(GimbalCtrl == gAim && DeviceState.PC_State == 1 && ReceiveVisionData.data.dis > 0.1f){
		Can2Send[1] = (int16_t)(Gimbal_Speed_pid_Pitch[AIM].Output + ReceiveVisionData.data.Ref_aPitch);	
		Can2Send[3] = (int16_t)(Gimbal_Speed_pid_Yaw[AIM].Output + ReceiveVisionData.data.Ref_aYaw);	
	} else if(GimbalCtrl == gStop){
    Can2Send[1] = Can2Send[2] = Can2Send[3] = 0;
	} else {
		if(goal_flag == 1){
			Can2Send[2] = (int16_t)(Gimbal_Speed_pid_UnderP[GYRO].Output);
			Can2Send[3] = (int16_t)(Gimbal_Speed_pid_Yaw[MECH].Output);				
		}else{
			Can2Send[2] = (int16_t)(Gimbal_Speed_pid_UnderP[GYRO].Output);
			Can2Send[3] = (int16_t)(Gimbal_Speed_pid_Yaw[GYRO].Output);		
		}
		Can2Send[1] = (int16_t)(Gimbal_Speed_pid_Pitch[GYRO].Output + GimbalPitchOffset);	
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
  static float Expect_UnderPitchInit = 0;
	static float Expect_PitchInit = 0;
  static float Expect_YawInit = 0;
  uint16_t Expect_UnderPitchRamp = UnderPitch.MchanicalAngle;
  uint16_t Expect_PitchRamp = GimPitch.MchanicalAngle;
  uint16_t Expect_YawRamp   = GimYaw.MchanicalAngle;
	/* 获得归中位置 */
	if(Time.GimbalInit < 100){
#if   Yaw_Mid_Right < Yaw_Mid_Left
        if ( (GimYaw.MchanicalAngle <= Yaw_Mid_Left) && (GimYaw.MchanicalAngle >= Yaw_Mid_Right) )
#elif Yaw_Mid_Right > Yaw_Mid_Left
        if ( (GimYaw.MchanicalAngle <= Yaw_Mid_Left) || (GimYaw.MchanicalAngle >= Yaw_Mid_Right) )
#endif
		MidMode = FRONT; else MidMode = BACK;
	} else {
		if (MidMode == FRONT) Expect_YawInit = QuickCentering( GimYaw.MchanicalAngle, Yaw_Mid_Front );
		else Expect_YawInit = QuickCentering( GimYaw.MchanicalAngle, Yaw_Mid_Back );
	}
	/* Yaw come mid */
	Expect_YawRamp = RAMP_float(Expect_YawInit,Expect_YawRamp,150); 
	PID_Calc(&Gimbal_Place_pid_Yaw[INIT],GimYaw.MchanicalAngle,Expect_YawRamp);
	PID_Calc(&Gimbal_Speed_pid_Yaw[INIT],GimYaw.Speed,Gimbal_Place_pid_Yaw[INIT].Output);
  limit( Gimbal_Speed_pid_Yaw[INIT].Output,DM4310_LIMIT,-DM4310_LIMIT);
	/* Pitch come mid*/	
	Expect_PitchInit = QuickCentering(GimPitch.MchanicalAngle,Pitch_Mid);
	Expect_PitchRamp = RAMP_float(Pitch_Mid,Expect_PitchRamp,50); 
	PID_Calc(&Gimbal_Place_pid_Pitch[INIT],GimPitch.MchanicalAngle,Expect_PitchRamp);
	PID_Calc(&Gimbal_Speed_pid_Pitch[INIT],GimPitch.Speed,Gimbal_Place_pid_Pitch[INIT].Output);
	limit( Gimbal_Speed_pid_Pitch[INIT].Output,DM4310_LIMIT,-DM4310_LIMIT );
	/* UnderPitch come mid */
	Expect_UnderPitchInit = QuickCentering(UnderPitch.MchanicalAngle,UnderPitch_Mid);
	Expect_UnderPitchRamp = RAMP_float(UnderPitch_Mid,Expect_UnderPitchRamp,50); 
	PID_Calc(&Gimbal_Place_pid_UnderP[INIT],UnderPitch.MchanicalAngle,Expect_UnderPitchRamp);
	PID_Calc(&Gimbal_Speed_pid_UnderP[INIT],UnderPitch.Speed,Gimbal_Place_pid_UnderP[INIT].Output);
	
	Can2Send[1] = (int16_t)Gimbal_Speed_pid_Pitch[INIT].Output;
	Can2Send[2] = (int16_t)Gimbal_Speed_pid_UnderP[INIT].Output;
	Can2Send[3] = (int16_t)Gimbal_Speed_pid_Yaw[INIT].Output;
	limit(Can2Send[GIMBAL_SUM],DM4310_LIMIT,-DM4310_LIMIT);	
#if GIMBAL_RUN
	DM_MotorSend(&hcan2,0x3FE,Can2Send);
#endif	
	if(Time.GimbalInit >= 1000){
		Time.GimbalInit = 0;
		GimbalInitFlag  = 0;
		Gimbal.YawInit   = Expect_YawInit;
		Gimbal.UnderPInit   = Expect_UnderPitchInit;
		Gimbal.PitchInit = Expect_PitchInit;

		Gimbal.increase[UnderP] = 0;
		Gimbal.increase[PITCH] = 0;
		Gimbal.increase[YAW]   = 0;
		Gimbal.Ref[YAW] = IMU.Angle_Yawcontinuous;
		Gimbal.Ref[UnderP] = UnderPitch.Angle_DEG;
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
			case 0x302 : 
					DM4310_Receive(&GimPitch,CAN2_Rxbuff);
					Feed_Dog(&Gimbal_Dog[PITCH]);
				break;
      case 0x303 : 
					DM4310_Receive(&UnderPitch,CAN2_Rxbuff);
					Feed_Dog(&Gimbal_Dog[UnderP]);
				break;
			case 0x304 : 
					DM4310_Receive(&GimYaw,CAN2_Rxbuff);
					Feed_Dog(&Gimbal_Dog[YAW]);
				break;
			default:	break;
    }
  }
}
