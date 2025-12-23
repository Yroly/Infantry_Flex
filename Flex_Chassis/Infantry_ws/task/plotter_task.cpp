#include "cmsis_os.h"
#include "plotter.h"
#include "INS_Task.h"
#include "chassis_task.h"

at::Plotter plotter(&huart10);

extern "C" void plotter_task(){
	while(true){
		osDelay(1);
	}
}