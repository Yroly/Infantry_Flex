#include "Shoot.h"
#include "Time.h"
#include "Function.h"
#include "Time.h"
#include "Gimbal.h"
#include "Music.h"
#include "VT03.h"

PID_TypeDef Single_Place_PID = {.Kp = 30, .Ki = 0, .Kd = 0, .Output = 5000};
PID_TypeDef Single_Speed_PID = {.Kp = 10, .Ki = 0, .Kd = 0, .Output = 5000};

PID Shoot_Speed_PID[FRIC_SUM] = {{.Kp = 15, .Ki = 0, .Kd = 0, .limit = 5000},
																 {.Kp = 15, .Ki = 0, .Kd = 0, .limit = 5000}};
PID_Smis Pluck_Place_PIDS = {.Kp = 30, .Ki = 0, .Kd = -0.8, .limit = 5000}; 
PID Pluck_Speed_PID = {.Kp = 10, .Ki = 0, .Kd = 0, .limit = 5000};                   
PID Pluck_Continue_PID = {.Kp = 20, .Ki = 0, .Kd = 0, .limit = 5000};               
int16_t Can1Send_Shoot[4] ={0};
struct SHOOT{	
	int16_t Ref_3508[FRIC_SUM];
	int16_t Ref_2006;
	int16_t Ref_2006_Angle;
	enum{
		SHOOT_STOP = 0,
		SHOOT_READY = 1,
		SHOOT_NORMAL = 2,
		SHOOT_RUNNING = 3,
		SHOOT_STUCKING = 4,
	}Action;
}SHOOT;
RM3508_TypeDef Shoot_Motor[FRIC_SUM];
M2006_TypeDef  Pluck_Motor;

uint8_t Add_Angle_Flag = 0;
uint8_t Shoot_One_Flag = 0;
uint8_t Lose_Angle_Flag = 0;
uint8_t	Running_Flag = 0;
float 	Angle_Target = 0;
float 	RAMP_Angle_Target = 0;
uint16_t tim = 0;
uint16_t Stuck_time = 0;
/**
 *@breif 发射控制模式
 */
void ShootCtrl_Decide(){   
	if(DeviceState.VT03_State == Device_Online){
		VT03.mode == ComInput ? Shoot_Key_Ctrl() :
		VT03.mode == RcInput ? Shoot_Rc_Ctrl() :
		Shoot_Close();
	}else Shoot_Close();
}
/**
 *@breif 遥控器控制
 */
void Shoot_Rc_Ctrl(){
	if(GimbalCtrl != gAim){
		if(VT03.trigger == 1){
			SHOOT.Action = SHOOT_READY;
			SHOOT.Action = SHOOT_RUNNING;	
		} else SHOOT.Action = SHOOT_STOP;			
	} else {
    SHOOT.Action = SHOOT_READY;
    if(ReceiveVisionData.data.FireFlag == 1 && VT03.trigger == 1){
      SHOOT.Action = SHOOT_RUNNING;
    } else {
      SHOOT.Action = SHOOT_READY;
    }
  }
}
/**
 *@breif 键鼠控制
 */
void Shoot_Key_Ctrl(){
	static char mouse_middle_flag = 0, Key_E_flag = 0;
  static uint16_t normal_time = 0,shoot_tim = 0;
	if(Referee_data_Rx.game_state == 1) SHOOT.Action = SHOOT_READY;
	else{
		if(VT03.keys.E == 1&& Key_E_flag == 0){
			if(SHOOT.Action != SHOOT_READY) SHOOT.Action = SHOOT_READY;
			else SHOOT.Action = SHOOT_STOP;
			Key_E_flag = 1;
		}
		if(VT03.keys.E == 0) Key_E_flag = 0;
	}
	if(GimbalCtrl != gAim){		
		if(VT03.mouse.left == 1)SHOOT.Action = SHOOT_RUNNING;
		if(SHOOT.Action == SHOOT_RUNNING && VT03.mouse.left == 0){
			SHOOT.Action = SHOOT_READY;											   
		}
	} else {
		SHOOT.Action = SHOOT_READY;
		if(ReceiveVisionData.data.dis > 0.1f){
			if(ReceiveVisionData.data.FireFlag == 0){
				if(VT03.mouse.left == 1)	SHOOT.Action = SHOOT_RUNNING;
			} else {
					if(Referee_data_Rx.game_state == 1) SHOOT.Action = SHOOT_RUNNING;
					else {
						if(VT03.mouse.left == 1) SHOOT.Action = SHOOT_RUNNING;
						else SHOOT.Action = SHOOT_READY;
					}
			}
		}
	}
}
void Shoot_Stop(){
	SHOOT.Ref_3508[LEFT]    =  0;
  SHOOT.Ref_3508[RIGHT]   =  0; 
	SHOOT.Ref_2006          =  0;
	PID_Control(Pluck_Motor.Measure.RoSpeed, SHOOT.Ref_2006, &Pluck_Speed_PID);
	PID_Control(Shoot_Motor[LEFT].Measure.RoSpeed, SHOOT.Ref_3508[LEFT], &Shoot_Speed_PID[LEFT]);
	PID_Control(Shoot_Motor[RIGHT].Measure.RoSpeed, SHOOT.Ref_3508[RIGHT], &Shoot_Speed_PID[RIGHT]);
	limit(Pluck_Speed_PID.pid_out, PLUCK_SPEED, -PLUCK_SPEED);
  limit(Shoot_Speed_PID[LEFT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
  limit(Shoot_Speed_PID[RIGHT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
	Can1Send_Shoot[0] = (int16_t)Pluck_Speed_PID.pid_out;       
	Can1Send_Shoot[2] = (int16_t)Shoot_Speed_PID[LEFT].pid_out;
	Can1Send_Shoot[1] = (int16_t)Shoot_Speed_PID[RIGHT].pid_out; 
#if SHOOT_RUN
  MotorSend(&hcan1, 0X200, Can1Send_Shoot);
#endif
}
void Shoot_Close(){
  Can1Send_Shoot[0] = 0;       
  Can1Send_Shoot[1] = 0;
	Can1Send_Shoot[2] = 0; 
#if SHOOT_RUN
	MotorSend(&hcan1, 0X200, Can1Send_Shoot);
#endif
}
void Shoot_SendDown(){
	if(DeviceState.Pluck_State != Device_Online || DeviceState.Shoot_State[LEFT] != Device_Online || DeviceState.Shoot_State[RIGHT] != Device_Online)
	 Gimbal_action.shoot_status = shoot_offline;
	else Gimbal_action.shoot_status = shoot_online;
	
	if(SHOOT.Action == SHOOT_STOP) Gimbal_action.shoot_mode = shoot_mode_stop;
	else if(SHOOT.Action == SHOOT_RUNNING && GimbalCtrl != gAim) Gimbal_action.shoot_mode = shoot_mode_fire;
	else if(GimbalCtrl == gAim) Gimbal_action.shoot_mode = shoot_mode_follow;
	
	if(SHOOT.Action == SHOOT_STUCKING)Gimbal_action.shoot_mode = shoot_mode_stucking;
} 
void ShootRef_Set(){
	SHOOT.Ref_3508[LEFT]  =  -SHOOT_SPEED;
	SHOOT.Ref_3508[RIGHT] =  SHOOT_SPEED; 
	switch(SHOOT.Action){
		case SHOOT_STOP:
			Shoot_Stop();
			Time.Single             = 0;
			SHOOT.Ref_2006_Angle    = Pluck_Motor.Measure.continueMechAngle;
			break;
		case SHOOT_READY:
			SHOOT.Ref_2006          =  0;
			Add_Angle_Flag			    =  1;
			Running_Flag            =  0;
			Pluck_Motor.Measure.r   =  0;
			Time.Single             =  0;
			SHOOT.Ref_2006_Angle    = Pluck_Motor.Measure.continueMechAngle;
			break;
		case SHOOT_NORMAL: break;
		case SHOOT_RUNNING:
			if(Referee_data_Rx.game_state == 1){
				ShootHeat_Limit();
			} else {
//				ShootHeat_Limit();
				SHOOT.Ref_2006 =   PLUCK_SPEED;
			}			break;
		case SHOOT_STUCKING:
			SHOOT.Ref_2006 =  - PLUCK_SPEED;
		break;
	}
}
void Shoot_Console(){
  if(SHOOT.Action == SHOOT_NORMAL){
    ShootSingle_Mode();
  }else{
    PID_Control(Pluck_Motor.Measure.RoSpeed, SHOOT.Ref_2006, &Pluck_Speed_PID);  
    limit(Pluck_Speed_PID.pid_out, M2006_LIMIT, -M2006_LIMIT);
    Can1Send_Shoot[0] = (int16_t)Pluck_Speed_PID.pid_out;  
  }
	PID_Control(Shoot_Motor[LEFT].Measure.RoSpeed, SHOOT.Ref_3508[LEFT], &Shoot_Speed_PID [LEFT]);
	PID_Control(Shoot_Motor[RIGHT].Measure.RoSpeed, SHOOT.Ref_3508[RIGHT], &Shoot_Speed_PID [RIGHT]);
	limit(Shoot_Speed_PID[LEFT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
	limit(Shoot_Speed_PID[RIGHT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
}
/**
 *@breif 发射机构单发模式
 */
uint16_t SingleTime = 0;
void ShootSingle_Mode(){
  static char AddSingle_Flag = 0,ShootSingle_Flag = 0;
  if(AddSingle_Flag == 0){
    SHOOT.Ref_2006_Angle = Pluck_Motor.Measure.angle + PLUCK_MOTOR_ONE;
    PID_Control_Smis(SHOOT.Ref_2006_Angle,Pluck_Motor.Measure.angle,&Pluck_Place_PIDS,Pluck_Motor.Measure.RoSpeed);
    PID_Control(Pluck_Motor.Measure.RoSpeed,Pluck_Place_PIDS.pid_out,&Pluck_Speed_PID);
    Can1Send_Shoot[0] = -(int16_t)Single_Speed_PID.Output;
    MotorSend(&hcan1,0x200,Can1Send_Shoot);
  }
  if(AddSingle_Flag == 1){
    SHOOT.Ref_2006_Angle = Pluck_Motor.Measure.angle;
    PID_Control_Smis(SHOOT.Ref_2006_Angle,Pluck_Motor.Measure.angle,&Pluck_Place_PIDS,Pluck_Motor.Measure.RoSpeed);
    PID_Control(Pluck_Motor.Measure.RoSpeed,Pluck_Place_PIDS.pid_out,&Pluck_Speed_PID);
    Can1Send_Shoot[0] = -(int16_t)Single_Speed_PID.Output;
    MotorSend(&hcan1,0x200,Can1Send_Shoot);
  }
  if(SingleTime > 300){
    AddSingle_Flag = 0;
    SingleTime = 0;
  }
}
/**
 *@breif 发射机构控制量发送
 */
void Shoot_Send(){
	Can1Send_Shoot[2] = (int16_t)Shoot_Speed_PID[LEFT].pid_out;
	Can1Send_Shoot[1] = (int16_t)Shoot_Speed_PID[RIGHT].pid_out; 
#if SHOOT_RUN
	if(SHOOT.Action != SHOOT_STOP)	MotorSend(&hcan1, 0X200, Can1Send_Shoot);
#endif
}
/**
 * @brief 计算电机转速与弹频的关系
 * @param target_freq 目标弹频 (Hz)，默认 20Hz
 * @return uint32_t 计算后的转速限制值 (RPM) * 
 * @note 转速最大为5400RPM时，弹频即可达到20Hz reduction_ratio 减速比 caliper 拨齿数 shoot_speed 弹频
 * @see SHOOT.Ref[2006] / 36 / 60 * 8 <= 20  转速 = 期望弹频 * 减速比 * 60s / 拨齿数
 * @warning 输入参数超出范围可能导致电机失控
 */
float cooling,heat_now,heat_limit,Consumption,shoot_speed,K = 2;
uint16_t ShootTime,shoot_time;
uint8_t reduction_ratio = 36.0,caliper = 12.0;
void ShootHeat_Limit(){
	heat_limit = Referee_data_Rx.heat_limit;
	cooling = Referee_data_Rx.heat_cooling;
	heat_now = Referee_data_Rx.heat_now;
	Consumption = 10.0f;//消耗
	if(heat_limit - heat_now > 100){
		shoot_speed = 15.0f;
	} else if (100 > (heat_limit - heat_now) && (heat_limit - heat_now) > 50){
		shoot_speed = (10.0f * heat_limit + 10.0f * cooling * shoot_time / 1000.0f - cooling) / (10.0f * Consumption * shoot_time / 1000.0f);	
	} else {
		shoot_speed =  cooling / Consumption;	
	}
	if( (GimbalCtrl == gAim && ReceiveVisionData.data.FireFlag == 1) || SHOOT.Action == SHOOT_RUNNING){
		shoot_time ++ ;	
	} else {
		shoot_time -- ;
	}
	SHOOT.Ref_2006 =   shoot_speed * reduction_ratio * 60.0f / caliper ;//转速 = 期望弹频 * 减速比 * 60s / 拨齿数
}
void Detect_Shoot(){
	if(SHOOT.Action != SHOOT_STOP){
		if(SHOOT.Action == SHOOT_RUNNING){
			if( ABS(Pluck_Motor.Measure.RoSpeed) <= 20)
				Stuck_time++;
		}
		if(Stuck_time >= 50){
			SHOOT.Action = SHOOT_STUCKING;
			Stuck_time++;
		}
		if( Stuck_time > 550){
			Stuck_time = 0;
			SHOOT.Action = SHOOT_RUNNING;
		}
	}
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  if (hcan->Instance == CAN1){
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader[0],CAN1_buff);
    switch (RxHeader[0].StdId){
			case 0x201: RMMotor_Receive(&Pluck_Motor.Measure, CAN1_buff);
									Feed_Dog(&Pluck_Dog);
									break;
			case 0x203: RMMotor_Receive(&Shoot_Motor[LEFT].Measure, CAN1_buff); 
									Feed_Dog(&Shoot_Dog[LEFT]);
									break;   
			case 0x202: RMMotor_Receive(&Shoot_Motor[RIGHT].Measure, CAN1_buff);
									Feed_Dog(&Shoot_Dog[RIGHT]);
									break; 
			default:    break;
		}
  }
}
