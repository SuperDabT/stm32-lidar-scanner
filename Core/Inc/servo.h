#ifndef SERVO_H
#define SERVO_H
#define SERVO_MIN_US 1000.0f
#define SERVO_MAX_US 2000.0f
#define SERVO_RANGE_DEG 180.0f
#define PAN_MAX 200.0f
#define PAN_MIN 0.0f
#define TILT_MIN 100.0f
#define TILT_MAX 130.0f /* Tilt: 130 = horizontal, lower = further up. Mechanical stop at 135. */
void servo_write(float angle,char axis);
void servo_init(void);
#endif
