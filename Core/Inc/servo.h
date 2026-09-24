#ifndef SERVO_H
#define SERVO_H

#define SERVO_MIN_US 500.0f /* Measured: 500-2500us gives a true 180 deg sweep. */
#define SERVO_MAX_US 2500.0f /* 1000-2000 gave only ~130 deg on these units. */
#define SERVO_RANGE_DEG 180.0f

/* Pan is expressed in TRUE BEARINGS, not servo commands — 90 is straight
   ahead along the base's forward axis. servo_write adds the offset. */
#define PAN_OFFSET_DEG 15.0f
#define PAN_MAX 165.0f /* 180+15 causes for overheating therefore 165 redemmed to be the safer option.*/ 
#define PAN_MIN 0.0f
#define PAN_STEP_DEG 2.0f
 /* True 0 -> command 15. */

#define TILT_MIN   89.0f   /* 22 deg above level. */
#define TILT_STEP_DEG -3.0f
#define TILT_MAX  125.0f   /* 14 deg below level — steeper and the beam hits floor inside 3m. */
#define TILT_LEVEL 111.0f  /* Measured, not assumed. Lower points up, higher down. */

#define SCAN_TILT_MIN 89.0f  /* Raster bounds. Stops at 116 because below */
#define SCAN_TILT_MAX 116.0f /* that the floor caps range under a metre.  */

void servo_write(float angle,char axis);
void servo_init(void);
#endif
