#ifndef TRACK_H
#define TRACK_H

#include <stdint.h>
#include <stdbool.h>
#include "detect.h"
#include "stm32f4xx_hal.h"

typedef enum{
STATE_SURVEY,
STATE_CONFIRMING,
STATE_TRACKING
}track_state_t;

typedef struct{
to_xy_t position;

to_xy_t first_seen;

float vy;// Unit: cm/s
float vx;// Unit: cm/s

track_state_t target_state;

uint8_t look_history;

uint32_t seen_at_ms;

}track_t;

track_t track_update(detection_t detection_array[],uint8_t sus_objects);

#endif