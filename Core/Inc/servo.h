#ifndef SERVO_H
#define SERVO_H
#define PAN_MAX 180.0f
#define PAN_MIN 0.0f
#define TILT_MIN 0.0f // TODO CHECK AFTER BRACKET ASSEMBLY
#define TILT_MAX 120.0f // TODO CHECK AFTER BRACKET ASSEMBLY
void servo_write(float angle,char axis);
void servo_home(void);
#endif
