#ifndef SERVO_H
#define SERVO_H
#define PAN_MAX 180.0f
#define PAN_MIN 0.0f
#define TILT_MIN 100.0f
#define TILT_MAX 130.0f /* Tilt: 130 = horizontal, lower = further up. Mechanical stop at 135. */
void servo_write(float angle,char axis);
void servo_init(void);
#endif
