#include  "servo.h"
#include "main.h"
#include "tim.h"


void servo_init(void){ 
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  servo_write(90.0f,'p' );
  servo_write(90.0f,'t' );
  HAL_Delay(500);
  for(int angle=90;angle>=0;angle-=5){
    servo_write(angle, 'p');
    servo_write(angle, 't');
    HAL_Delay(50);
  }

}
void servo_write(float angle, char axis) {
	if (axis != 'p' && axis != 't') {
		return;
	}
	if ((axis == 'p') && (angle > PAN_MAX || angle < PAN_MIN)) {
		return;
	} else if ((axis == 't') && (angle > TILT_MAX || angle < TILT_MIN)) {
		return;
	}
	uint32_t pulse_us;
	pulse_us = 1000 + (angle / 180.0f) * 1000;

	if (axis == 't') {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us); //CH1/PA0 is tilt
	} else if (axis == 'p') {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_us); //CH2/PA1 is pan
	}

}