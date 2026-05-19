#include <motor.h>
/**
 * @brief 用转矩电流计算得到功率值
 * @param[in] speed 电机速度
 * @param[in] current 转矩电流
 * @param[in] pcof 参数
 * @return 电机功率
 */
static inline float GetChassisMotorPower(int speed, int current, struct PowerCOF_s *pcof) {
    return (pcof->ss * speed * speed +
            pcof->sc * speed * current +
            pcof->cc * current * current +
            pcof->constant);
}
void RMMotor_Receive(RMMotor_t *motor,uint8_t *Data){
	motor->MechAngle = (uint16_t)(Data[0] << 8 | Data[1]);
	motor->RoSpeed = (int16_t)(Data[2] << 8 | Data[3]);
	motor->TorCurrent = (uint16_t)(Data[4] << 8 | Data[5]);
	motor->Temp = Data[6];
	motor->Err =  Data[7];
	int16_t diff = motor->MechAngle - motor->lastMechAngle;
		if (diff > 4096)
				motor->r--;
		if (diff < -4096)
				motor->r++;
	motor->continueMechAngle = motor->r * 8192 + motor->MechAngle;
	motor->lastMechAngle = motor->MechAngle;
	motor->angle = motor->continueMechAngle * 0.0439453125f;
}
//	Dst->Power = GetChassisMotorPower(Dst->Speed, Dst->TorqueCurrent, &Dst->PowerCOF);

HAL_StatusTypeDef MotorSend(CAN_HandleTypeDef *hcan, uint32_t StdId, int16_t *Data) {
    uint8_t temp[8];
    temp[0] = (uint8_t)(Data[0] >> 8);
    temp[1] = (uint8_t)(Data[0] & 0xff);
    temp[2] = (uint8_t)(Data[1] >> 8);
    temp[3] = (uint8_t)(Data[1] & 0xff);
    temp[4] = (uint8_t)(Data[2] >> 8);
    temp[5] = (uint8_t)(Data[2] & 0xff);
    temp[6] = (uint8_t)(Data[3] >> 8);
    temp[7] = (uint8_t)(Data[3] & 0xff);
    return CAN_Send_StdDataFrame(hcan, StdId, temp);
}

int16_t QuickCentering(uint16_t current, uint16_t target){
	int16_t diff = (target + 4095) % 8192;
	if (diff < target) 
    return current < diff ? target - 8192 : target; 
  else
    return current > diff ? target + 8192 : target;
}
