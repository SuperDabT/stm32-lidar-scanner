#ifndef SERVO_H
#define SERVO_H

#define SERVO_MIN_US 500.0f /* Measured: 500-2500us gives a true 180 deg sweep. */
#define SERVO_MAX_US 2500.0f /* 1000-2000 gave only ~130 deg on these units. */
#define SERVO_RANGE_DEG 180.0f

#define PAN_MAX 180.0f /* Servo reaches ~210 but past 180 is unspecified extrapolation. */
#define PAN_MIN 0.0f

#define TILT_MIN 89.0f /* 22 deg up — enough for a close-range person, not ceiling. */
#define TILT_MAX 125.0f /* 80 = up, 111.3 = level, 130 = down. */
#define TILT_LEVEL 111.3f /* 14 deg down — steeper and the beam hits floor inside 3m. */

#define SCAN_TILT_MIN 89.0f  /* Raster bounds. Stops at 116 because below */
#define SCAN_TILT_MAX 116.0f /* that the floor caps range under a metre.  */

void servo_write(float angle,char axis);
void servo_init(void);
#endif
