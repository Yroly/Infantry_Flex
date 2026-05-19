#include "cmsis_os.h"
#include "plotter.h"
#include "ins_task.h"
#include "Gimbal.h"
#include "dm_motor.h"

at::Plotter plotter(&huart1);

extern "C" void Plotter_Task(){
	while(true){
		plotter.plot(IMU.Angle_Pitch,Gimbal.Ref[PITCH],UnderPitch.Angle_DEG,Gimbal.Ref[UnderP]);
		osDelay(10);
	}
}