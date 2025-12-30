#pragma once 
#include "pid.h"
#include "dvc_unitree.h"
#include "pid.h"
#include "remote_task.h"
#include "Chassis.h"
#define JOINT_RUN 1

typedef struct{
	PidTypeDef joint_spd[2];
	PidTypeDef joint_pos[2];
	PidTypeDef pitch;
}JointPid_t;

class Joint_Class{
public:
	JointPid_t joint_pid;
	void init(MotorCmd_t *cmd, unsigned short ID, float t, float w, float pos, float k_p, float k_w);
	void decide();
	void rc_ctrl();
	void key_ctrl();
	void stop();
	void control();
	void control_loop();
	void exchange();
};
extern Joint_Class Joint;
