#include "Chassis.h"

Chassis_Class Chassis;

void Chassis_Class::speed_clean(ChassisSpeed_Ref_t *ref){
	ref->forward_back_ref = 0;
	ref->left_right_ref = 0;
	ref->rotate_ref = 0;
}
void Chassis_Class::speed_get(MotorSpeed_Ref_t *motor, ChassisSpeed_Ref_t *ref){
    motor->speed_1 = ref->forward_back_ref + ref->left_right_ref + ref->rotate_ref;
    motor->speed_2 = -ref->forward_back_ref + ref->left_right_ref + ref->rotate_ref;
    motor->speed_3 = -ref->forward_back_ref - ref->left_right_ref + ref->rotate_ref;
    motor->speed_4 = ref->forward_back_ref - ref->left_right_ref + ref->rotate_ref;
}