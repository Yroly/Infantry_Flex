#include "remote_task.h"

extern "C" void Remote_Task(){
	static portTickType currentTime;
	remote.init(&huart5,false);
	dog.init(&dog.Remote_Dog,20);
	remote.request();
	for(;;){
		currentTime = xTaskGetTickCount();
		dog.polling();
		osDelay(15);
	}
}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size){
  if (huart == &huart5) {
		remote.sbus_to_rc();			
		dog.feed(&dog.Remote_Dog);
    remote.request();
  }
	if(huart == &huart2){
		Unitree.extract_data(&Unitree.measure[0]);
		dog.feed(&dog.Joint_Dog[0]);
	}
	if(huart == &huart3){
		Unitree.extract_data(&Unitree.measure[1]);
		dog.feed(&dog.Joint_Dog[1]);
	}
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart){
  if (huart == &huart5) {
    remote.request();
  }
}