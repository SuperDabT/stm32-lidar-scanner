#include "detect.h"
#include "servo.h"
#include <stdbool.h>
#include <stdint.h>
#include <sys/_intsup.h>

#define PAN_SLOTS (((int)(PAN_MAX - PAN_MIN) / (int)PAN_STEP_DEG) + 1)
#define BASELINE_SWEEPS 3
#define MIN_DROP_CM 40
#define MAX_OBJECTS 4
#define MIN_RUN_LENGTH 3

static uint16_t background[PAN_SLOTS][BASELINE_SWEEPS];
static uint16_t baseline[PAN_SLOTS];
static uint8_t sweep_counter = 0;
static uint8_t dropped_counter = 0;

static uint16_t first_bearing;
static uint16_t last_bearing;
static uint16_t first_dist;
static uint16_t last_dist;
static uint8_t readings_count = 0;
static uint16_t min_dist;

static detection_t objects[MAX_OBJECTS];
static uint8_t object_count = 0;
static uint16_t dropped_counter = 0

    static bool calibrating = true

    static uint16_t
    median3(uint16_t a, uint16_t b, uint16_t c) {
  uint16_t temp_val = 0;

  if (a > b) {
    temp_val = a;
    a = b;
    b = temp_val;
  }
  if (a > c) {
    temp_val = c;
    c = a;
    a = temp_val;
  }
  if (b > c) {
    temp_val = c;
    c = b;
    b = temp_val;
  }
  return b;
}

void detect_calibrate(void) {
  sweep_counter = 0;
  calibrating = true;
}

void detect_feed(uint16_t bearing, uint16_t distance) {
  uint16_t slot = (uint16_t)((bearing - PAN_MIN) / PAN_STEP_DEG);
  if (slot >= PAN_SLOTS) {
    return;
  }

  if (calibrating) {
    if (sweep_counter >= BASELINE_SWEEPS) {
      return;
    }
    background[slot][sweep_counter] = distance;
  } else {
    if (distance + MIN_DROP_CM < baseline[slot]) {
      if (readings_count == 0) {
        first_bearing = bearing;
        first_dist = distance;
        min_dist = first_dist;
      }
      last_bearing = bearing;
      last_dist = distance;
      if (distance < min_dist) {
        min_dist = distance;
      }
      readings_count++;

    } else {
      if (readings_count >= MIN_RUN_LENGTH) {
        if (object_count < MAX_OBJECTS) {
          objects[object_count].found = true;
          objects[object_count].distance = min_dist;
          objects[object_count].width = 0;
          objects[object_count].bearing = (last_bearing + first_bearing) / 2.0f;

          object_count++;
        } else {
          dropped_counter++;
        }
      }
      readings_count = 0;
    }
  }
}

void detect_sweep_end(void) {
  if (calibrating) {
    sweep_counter++;
    if (sweep_counter == BASELINE_SWEEPS) {
      for (uint8_t i = 0; i < PAN_SLOTS; i++) {
        baseline[i] =
            median3(background[i][0], background[i][1], background[i][2]);
      }
      calibrating = false;
    }
  }
}