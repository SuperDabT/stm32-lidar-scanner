#include "detect.h"
#include "servo.h"
#include <stdint.h>

#define PAN_SLOTS (((int)(PAN_MAX-PAN_MIN) / (int)PAN_STEP_DEG) + 1)
#define BASELINE_SWEEPS 3

static uint16_t background[PAN_SLOTS][BASELINE_SWEEPS];
static uint8_t sweep_counter;

