#ifndef DETECT_H
#define DETECT_H

#include <stdint.h>
#include <stdbool.h>
typedef struct{

    bool found;
    uint16_t distance;
    uint16_t width;
    float bearing;

}detection_t;


void detect_calibrate(void);

void detect_feed(uint16_t bearing,uint16_t distance);

detection_t detect_result (void);

void detect_sweep_end(void);
#endif