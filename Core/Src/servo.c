#include  "servo.h"
#include "main.h"
#include "tim.h"


void servo_init(void){ 
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  servo_write(0.0f,'p' );
  servo_write(115.0f,'t' );  /* 115 is level */
  HAL_Delay(500);  /* let both axes reach home before scanning */
 

}
void servo_write(float angle, char axis) {
	if (axis != 'p' && axis != 't') {
		return;
	}
	 /* Out-of-range angles are rejected outright rather than clamped, so a
       caller asking for something impossible gets no movement rather than
       silently getting the limit. */
	if ((axis == 'p') && (angle > PAN_MAX || angle < PAN_MIN)) {
		return;
	} else if ((axis == 't') && (angle > TILT_MAX || angle < TILT_MIN)) {
		return;
	}

	/* TIM2 is prescaled to 1 MHz, so one timer tick is one microsecond and
       the compare register holds the pulse width directly. */

	uint32_t pulse_us;
	pulse_us = SERVO_MIN_US+ (angle/ SERVO_RANGE_DEG) * (SERVO_MAX_US-SERVO_MIN_US);

	if (axis == 't') {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us); //CH1/PA0 is tilt
	} else if (axis == 'p') {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_us); // CH2/PA1 is pan
	}

}