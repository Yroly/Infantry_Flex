#include "uart_task.h"
#include "referee_unpack.h"
#include "plotter.h"
at::Plotter plotter(&huart10);

extern "C" void UART_Task(){
	static portTickType currentTime;
	remote.init(&huart5,false);
	Referee.init(&huart1,false);
	dog.init(&dog.Remote_Dog,20);
	remote.request();
	Referee.request();
	for(;;){
		currentTime = xTaskGetTickCount();
		dog.polling();
		osDelay(15);
	}
}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size){
  if(huart == &huart5) {
		remote.sbus_to_rc();			
		dog.feed(&dog.Remote_Dog);
    remote.request();
  }
	if(huart == &huart1){
		Referee.update(Size);
		Referee.request();
	}
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart){
  if (huart == &huart5) {
    remote.request();
  }
	if(huart == &huart1){
		Referee.request();
	}
}