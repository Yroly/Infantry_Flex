#include "pid.h"

PID_Class PID;



void PID_Class::Init(PidTypeDef *pid,fp32 Kp,fp32 Ki,fp32 Kd,fp32 Kf,fp32 max){
	if (pid == NULL){
			return;
	}
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->Kf = Kf;
	pid->max = max;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->error[0] = pid->error[1] = pid->error[2] = pid->P_out = pid->I_out = pid->D_out = pid->out = 0.0f;
}

fp32 PID_Class::Calc(PidTypeDef *pid, fp32 measure,fp32 target){
	if (pid == NULL){
			return 0.0f;
	}

	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->measure = measure;
	pid->target	 = target;
	pid->error[0] = target - measure;
	pid->P_out = pid->Kp * pid->error[0];
	pid->I_out += pid->Ki * pid->error[0];
	pid->Dbuf[2] = pid->Dbuf[1];
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
	pid->D_out = pid->Kd * pid->Dbuf[0];
	pid->F_out = pid->Kf * (pid->measure - pid->last_measure);
	pid->out = pid->P_out + pid->I_out + pid->D_out + pid->F_out;
	pid->last_measure = pid->measure;
	return pid->out;
}

void PID_Class::Clear(PidTypeDef *pid){
	if (pid == NULL){
			return;
	}

	pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->out = pid->P_out = pid->I_out = pid->D_out = 0.0f;
	pid->measure = pid->target = 0.0f;
}
