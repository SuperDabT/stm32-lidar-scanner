#ifndef SERVO_H
#define SERVO_H
#define SERVO_MIN_US 500.0f /* Measured: 500-2500us gives a true 180 deg sweep. */
#define SERVO_MAX_US 2500.0f /* 1000-2000 gave only ~130 deg on these units. */
#define SERVO_RANGE_DEG 180.0f
#define PAN_MAX 180.0f /* Servo reaches ~210 but past 180 is unspecified extrapolation. */
#define PAN_MIN 0.0f
#define TILT_MIN 80.0f
#define TILT_MAX 115.0f /* 80 = up, 115 = level, 130 = down. */
void servo_write(float angle,char axis);
void servo_init(void);
#endif
