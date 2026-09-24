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

}detection_t;


void detect_calibrate(void);

void detect_feed(uint16_t bearing,uint16_t distance);

uint8_t detect_result (detection_t detection_array[], uint8_t max_slots);

void detect_sweep_end(void);
#endif