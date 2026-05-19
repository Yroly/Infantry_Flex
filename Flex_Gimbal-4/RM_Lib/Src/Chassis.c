#include "Chassis.h"
#include "arm_math.h"
#include "rng.h"
#include "dm_motor.h"
#include "Gimbal.h"
#include "VT03.h"

PID_TypeDef Chassis_Place_pid_Rotate;
PID_TypeDef Chassis_Speed_pid_Rotate;
FeedForward_Typedef Chassis_FF = {.K1 = 1000.00, .OutMax = RM3508_LIMIT};        //前馈
uint16_t Mid_Left,Mid_Right,Mid_Back,Mid_Front;

static	Gimbal_board_send_t send_data;
struct{
enum{
ChassisStop = 0,
ChassisFollow = 1,
ChassisNormal = 2,
ChassisGyroscope = 3
}Action;
int16_t MidAngle;
}CHASSIS;
/**
*@brief up to under 
*/
Communication_Speed_t Communication_Speed_Tx;
/**
*@brief chassis init
*/
void ChassisInit(){
  Mid_Front = Yaw_Mid_Front;
  PID_init(&Chassis_Place_pid_Rotate,8000,5,0, 1,0,0,0,0.001);
  PID_init(&Chassis_Speed_pid_Rotate,8000,5,0, 2,0,0,0,0.001);
  CHASSIS.Action = ChassisFollow;
}
/**
*@brief chassis ctrl decicde
*/
void ChassisCtrl_Decide(){
	if(DeviceState.VT03_State == Device_Online){
		VT03.mode == ComInput ? Chassis_Key_Ctrl() :
		VT03.mode == RcInput ? Chassis_RC_Ctrl() :
		Chassis_Stop();
	}else Chassis_Close();
}
/**
*@brief chassis rc ctrl
*/
void Chassis_RC_Ctrl(){
  static char rc_pause_flag = 0;
	Communication_Speed_Tx.Close_flag = 0;
  
  /* 按下pause键小陀螺 */
  if(VT03.pause == 1 && rc_pause_flag == 0){
    if(CHASSIS.Action != ChassisGyroscope) CHASSIS.Action = ChassisGyroscope;
    else CHASSIS.Action = ChassisFollow;
    rc_pause_flag = 1;
  }
  if(VT03.pause == 0) rc_pause_flag = 0;
  /*自瞄模式下底盘状态*/
  if(GimbalCtrl == gAim){
    if(CHASSIS.Action != ChassisGyroscope ) CHASSIS.Action = ChassisNormal;
    else CHASSIS.Action = ChassisGyroscope ;
  } else {
    if(CHASSIS.Action == ChassisStop || CHASSIS.Action == ChassisNormal) CHASSIS.Action = ChassisFollow;
  }
}
/**
*@brief WSAD 前后左右
*@brief r 小陀螺
*@brief F 转向180
*/
static char Key_F_flag = 0;
void Chassis_Key_Ctrl(){
	static char Key_R_flag = 0,Key_Ctrl_flag = 0;
	static uint16_t tim;
	tim ++;
	Communication_Speed_Tx.Close_flag = 0;			
  /* WS前后 */
	if(VT03.keys.W) VT03.ch_ry = 1;
	else if(VT03.keys.S) VT03.ch_ry = -1;
	else VT03.ch_ry = 0;
  /* AD左右 */
	if(VT03.keys.A) VT03.ch_rx = -1;
	else if(VT03.keys.D) VT03.ch_rx = 1;
	else VT03.ch_rx = 0;    
  /* R键按下小陀螺 */
	if(VT03.keys.R == 1 && Key_R_flag == 0){
		if(CHASSIS.Action != ChassisGyroscope) CHASSIS.Action = ChassisGyroscope;
		else CHASSIS.Action = ChassisFollow;
		Key_R_flag = 1;
	}
	if (VT03.keys.R == 0) Key_R_flag = 0;
	/* 按下F键转头180 */
	if (VT03.keys.F == 1 && Key_F_flag == 0){
    Key_F_flag = 1;
    tim = 0;
	}	else if (Key_F_flag == 1 && tim > 500){
    if(VT03.keys.F == 0 && Key_F_flag == 1){
      Key_F_flag = 0;
    }
  }
  /* 自瞄模式下底盘状态 */
	if(GimbalCtrl == gAim){
		if(CHASSIS.Action != ChassisGyroscope) CHASSIS.Action = ChassisNormal;
    else CHASSIS.Action = ChassisGyroscope;
		if(VT03.keys.Ctrl == 1) CHASSIS.Action = ChassisFollow;
	} else {
    if(CHASSIS.Action == ChassisStop || CHASSIS.Action == ChassisNormal)CHASSIS.Action = ChassisFollow;
  }
}
void Chassis_Stop(){
	CHASSIS.Action = ChassisStop;
}
void ChassisRef_Update(){
	static uint16_t Follow_Speed_MAX = 5000 * PI;		 
	static uint16_t Speed,tim;		 
	static uint16_t Ramp_rotate_ref;	
	switch(CHASSIS.Action){
		case ChassisFollow:
			Chassis_FF.Now_DeltIn = VT03.ch_lx + VT03.mouse.vx * 0.8;
#if   Yaw_Mid_Right < Yaw_Mid_Left
        if ( (GimYaw.MchanicalAngle <= Yaw_Mid_Left) && (GimYaw.MchanicalAngle >= Yaw_Mid_Right) ){
#elif Yaw_Mid_Right > Yaw_Mid_Left
        if ( (GimYaw.MchanicalAngle <= Yaw_Mid_Left) || (GimYaw.MchanicalAngle >= Yaw_Mid_Right) ){
#endif
        CHASSIS.MidAngle = Yaw_Mid_Front; 
        MidMode = FRONT;
    } else {
        CHASSIS.MidAngle = Yaw_Mid_Back;
        MidMode = BACK;
    }
        PID_Calc(&Chassis_Place_pid_Rotate,GimYaw.MchanicalAngle,QuickCentering(GimYaw.MchanicalAngle, CHASSIS.MidAngle));
				PID_Calc(&Chassis_Speed_pid_Rotate,GimYaw.Speed,Chassis_Place_pid_Rotate.Output);
        Chassis_Speed_pid_Rotate.Output -= Chassis_Place_pid_Rotate.Output;//前馈
				Communication_Speed_Tx.Chassis_Speed.rotate_ref = Chassis_Speed_pid_Rotate.Output + FeedForward_Calc(&Chassis_FF) + VT03.wheel * 2000 * PI;
				limit(Communication_Speed_Tx.Chassis_Speed.rotate_ref, Follow_Speed_MAX, -Follow_Speed_MAX);
	      if(Key_F_flag) Communication_Speed_Tx.Chassis_Speed.rotate_ref = 0;
			break;
				
		case ChassisNormal:
			Ramp_rotate_ref = VT03.wheel * 2000 * PI ;
			Communication_Speed_Tx.Chassis_Speed.rotate_ref   = RAMP_float(Ramp_rotate_ref,Communication_Speed_Tx.Chassis_Speed.rotate_ref,15) ;
		break;	

		case ChassisGyroscope:
			if(VT03.ch_rx == 0 && VT03.ch_ry == 0 && VT03.keys.Shift == 0){
				Ramp_rotate_ref = 2000 * PI;
				Communication_Speed_Tx.Chassis_Speed.rotate_ref = RAMP_float(Ramp_rotate_ref,Communication_Speed_Tx.Chassis_Speed.rotate_ref,15) ;	
			}			
		break;	
		case ChassisStop:
			Communication_Speed_Tx.Chassis_Speed.rotate_ref       = 0;
			Communication_Speed_Tx.Chassis_Speed.forward_back_ref = 0;		
			Communication_Speed_Tx.Chassis_Speed.left_right_ref   = 0;
			Communication_Speed_Tx.Close_flag = 1;				
		break;
  }
}
uint16_t R_C,L_C;
/*底盘补偿计算*/
void Chassis_Offset(){
	static float Level_Gain, chassis_offset;
	static int16_t forward_back_ref = 0, left_right_ref = 0,rotate_ref = 0;
	static int16_t Speed_Gain;
	static float Ramp_forward_back_ref,Ramp_left_right_ref,Ramp_rotate_ref;	
  if(MidMode == FRONT){
    chassis_offset = (Yaw_Mid_Front - GimYaw.MchanicalAngle) / 1303.64f;//底盘补偿角
    Gimbal_action.bof = 0;
  } else if(MidMode == BACK){
    chassis_offset = (Yaw_Mid_Back - GimYaw.MchanicalAngle) / 1303.64f;//底盘补偿角
    Gimbal_action.bof = 1;  
  }
	Gimbal_data.Offset_Angle = chassis_offset * 1000;
	R_C = Yaw_Mid_Right;
	L_C = Yaw_Mid_Left;
	/* 按住Shift加速 */
	if(VT03.keys.Shift){
		Speed_Gain = 7000; 
	} else Speed_Gain = 4000;
  
  if(MidMode == FRONT){
  forward_back_ref = -VT03.ch_ry * Speed_Gain;
  left_right_ref   = -VT03.ch_rx * Speed_Gain * 0.7;
  }else if(MidMode == BACK){
  forward_back_ref = VT03.ch_ry * Speed_Gain;
  left_right_ref   = VT03.ch_rx * Speed_Gain * 0.7;
  }
	
  if(CHASSIS.Action == ChassisGyroscope && (VT03.ch_rx || VT03.ch_ry || VT03.keys.Shift == 1)){
    rotate_ref = 1000 * PI;
    Ramp_rotate_ref = RAMP_float(rotate_ref,Ramp_rotate_ref,Speed_Gain/750.0); 
    Communication_Speed_Tx.Chassis_Speed.rotate_ref = RAMP_float(rotate_ref,Communication_Speed_Tx.Chassis_Speed.rotate_ref,Speed_Gain/750.0); 
  }		                                  	
  /* 底盘补偿计算 小陀螺移动 只有YAW轴在线才可解算 */
  if(DeviceState.Gimbal_State[YAW] == Device_Online && CHASSIS.Action == ChassisGyroscope){
    Communication_Speed_Tx.Chassis_Speed.left_right_ref =     - forward_back_ref * arm_sin_f32(-chassis_offset)
                                                              + left_right_ref * arm_cos_f32(-chassis_offset);
    Communication_Speed_Tx.Chassis_Speed.forward_back_ref  =    forward_back_ref * arm_cos_f32(-chassis_offset) 
                                                              + left_right_ref * arm_sin_f32(-chassis_offset);
  } else {
    Communication_Speed_Tx.Chassis_Speed.forward_back_ref =  forward_back_ref;
    Communication_Speed_Tx.Chassis_Speed.left_right_ref   =  left_right_ref;
  }
}
void Chassis_Close(){
	Communication_Speed_Tx.Chassis_Speed.rotate_ref       = 0;
	Communication_Speed_Tx.Chassis_Speed.forward_back_ref = 0;
	Communication_Speed_Tx.Chassis_Speed.left_right_ref   = 0;
	Communication_Speed_Tx.Close_flag = 1;

	send_data.vx         = Communication_Speed_Tx.Chassis_Speed.forward_back_ref;
	send_data.vy         = Communication_Speed_Tx.Chassis_Speed.left_right_ref;
	send_data.rotate     = Communication_Speed_Tx.Chassis_Speed.rotate_ref;
	send_data.Close_flag = Communication_Speed_Tx.Close_flag;
	send_data.Shift_flag = 0;
#if CHASSIS_RUN
    CAN_Send_StdDataFrame(&hcan2, 0x110, (uint8_t *)&send_data);
#endif
}
void ChassisDown_Send(){
  static char Key_Q_flag = 0;
	send_data.vx         = Communication_Speed_Tx.Chassis_Speed.forward_back_ref;
	send_data.vy         = Communication_Speed_Tx.Chassis_Speed.left_right_ref;
	send_data.rotate     = Communication_Speed_Tx.Chassis_Speed.rotate_ref;
	send_data.Close_flag = Communication_Speed_Tx.Close_flag;
	send_data.Shift_flag = 0;
	switch(CHASSIS.Action) {
		case ChassisStop: Gimbal_action.move_status = stop; break;
		case ChassisNormal: Gimbal_action.move_status = normal; break;
		case ChassisGyroscope: Gimbal_action.move_status = rotate; break;
		case ChassisFollow:
      if(VT03.mode == RcInput) Gimbal_action.move_status = follow;
      if(VT03.mode == ComInput){
        if(VT03.keys.Q == 1 && Key_Q_flag == 0){
          if(Gimbal_action.move_status != Auto) Gimbal_action.move_status = Auto;
          else Gimbal_action.move_status = follow;
          Key_Q_flag = 1;
        }
        if(VT03.keys.Q == 0){
          if(Gimbal_action.move_status != Auto) Gimbal_action.move_status = follow;
          Key_Q_flag = 0;
        } 
           
      }
      break;	
	}   
#if CHASSIS_RUN
    CAN_Send_StdDataFrame(&hcan2, 0x110, (uint8_t *)&send_data);	
#endif
}
void ChassisMotorSpeed_clean(ChassisSpeed_Ref_t *ref) {
    ref->forward_back_ref = 0;
    ref->left_right_ref = 0;
    ref->rotate_ref = 0;
}
__weak void ChassisMotorSpeed_get(Chassis_Motor_Speed *motor, ChassisSpeed_Ref_t *ref) {
    motor->speed_3 = -ref->forward_back_ref -
                     ref->left_right_ref + ref->rotate_ref;

    motor->speed_2 = ref->forward_back_ref -
                     ref->left_right_ref + ref->rotate_ref;

    motor->speed_1 = ref->forward_back_ref +
                     ref->left_right_ref + ref->rotate_ref;

    motor->speed_4 = -ref->forward_back_ref +
                     ref->left_right_ref + ref->rotate_ref;
}
