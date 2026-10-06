#ifndef DETECT_H
#define DETECT_H

#include <stdint.h>
#include <stdbool.h>

/* How many candidates one sweep can hand to the tracker. */
#define MAX_OBJECTS 4

typedef struct{

    bool found;
    uint16_t distance;
    uint16_t width;
    float bearing;
    uint32_t time_stamp_ms;

}detection_t;

typedef struct{
float x;
float y;
}to_xy_t;

to_xy_t convert_xy (float bearing,uint16_t distance);

detection_t detect_latest(void);

void detect_calibrate(void);

bool detect_feed(uint16_t bearing,uint16_t distance,uint32_t time_ms);

uint8_t detect_result (detection_t detection_array[], uint8_t max_slots);

void detect_sweep_end(void);
#endif