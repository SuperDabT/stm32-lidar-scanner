/* detect.c — finds things in the room that weren't there when it was empty.

   Two phases:
   1. CALIBRATING: the room is empty. Record 3 sweeps, take the middle
      value at each angle. That's the "before photo" (baseline).
   2. WATCHING: compare every new reading to the before photo. A run of
      readings that are much closer than normal is a candidate object.

   Detection only finds candidates. It has no memory between sweeps, so
   it can't tell a person from a moved chair; the tracker decides that. */

#include "detect.h"
#include "servo.h"
#include <stdbool.h>
#include <stdint.h>

/* ---- Settings ---------------------------------------------------------- */

/* One slot per angle the head stops at: 0, 2, 4 ... so (range / step) + 1.
   Tied to the servo constants, so changing the step resizes everything. */
#define PAN_SLOTS ((((int)PAN_MAX - (int)PAN_MIN) / (int)PAN_STEP_DEG) + 1)

/* Sweeps recorded during calibration. Must be odd: the median of an even
   count averages the middle two, so one bad reading blends in instead of
   being outvoted. 3 is the smallest count that protects you. */
#define BASELINE_SWEEPS 3

/* How much closer than normal a reading must be to count as "something
   there". Measured: below 15 the sensor's own wobble trips it, above 65
   a person gets missed. 40 is the middle of that band. */
#define MIN_DROP_CM 40

/* Fewest readings in a row before a run is believed. Shorter = glitch. */
#define MIN_RUN_LENGTH 3

/* ---- Calibration memory ------------------------------------------------ */

static uint16_t background[PAN_SLOTS][BASELINE_SWEEPS]; /* scratch pad: 3 raw readings per angle */
static uint16_t baseline[PAN_SLOTS];                    /* finished before photo: 1 per angle */
static uint8_t  sweep_counter = 0;                      /* which calibration column we're filling */
static bool     calibrating   = true;                   /* start by learning the room */

/* ---- The run currently being built --------------------------------------
   Instead of storing every reading in a run, keep just what we need:
   where it started, where it ended, the closest point, and how long.   */

static uint16_t first_bearing;
static uint16_t first_dist;
static uint16_t last_bearing;
static uint16_t last_dist;
static uint16_t min_dist;            /* closest point: ignores the half-on-the-wall edges */
static uint8_t  readings_count = 0;  /* 0 means no run is going */

/* ---- Finished runs, waiting for main.c to collect them ------------------ */

static detection_t objects[MAX_OBJECTS];
static uint8_t     object_count    = 0;  /* how many stored = index of next free slot */
static uint16_t    dropped_counter = 0;  /* runs lost because all slots were full */

/* ---- Helpers ------------------------------------------------------------ */

/* Middle value of three. Sorts them by swapping, then b is the middle. */
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  uint16_t temp_val = 0;

  if (a > b) { temp_val = a; a = b; b = temp_val; }
  if (a > c) { temp_val = c; c = a; a = temp_val; }
  if (b > c) { temp_val = c; c = b; b = temp_val; }
  return b;
}

/* A run just ended. If it was long enough, turn it into a candidate and
   put it on the shelf. Either way, start fresh for the next run.
   Called from two places: a "not close" reading, and the end of a sweep. */
static void close_run(void) {
  if (readings_count >= MIN_RUN_LENGTH) {       /* real, not a glitch */
    if (object_count < MAX_OBJECTS) {           /* room on the shelf */
      objects[object_count].found    = true;
      objects[object_count].distance = min_dist;
      objects[object_count].width    = 0;       /* TODO: needs to_xy */
      objects[object_count].bearing  = (last_bearing + first_bearing) / 2.0f; /* middle */
      object_count++;
    } else {
      dropped_counter++;                        /* shelf full, count the loss */
    }
  }
  readings_count = 0;                           /* no run going anymore */
}

/* ---- Public functions (declared in detect.h) --------------------------- */

/* Start (or restart) learning the room. Room must be empty.
   Call only with the head at PAN_MIN, about to begin a sweep: if
   calibration starts mid-sweep, the first column misses some angles,
   their median comes out 0, and those angles go permanently blind. */
void detect_calibrate(void) {
  sweep_counter = 0;
  calibrating = true;
  object_count=0;
  readings_count=0;
}

/* Called by main.c once per reading, every 50 ms. */
void detect_feed(uint16_t bearing, uint16_t distance) {
  /* Turn the angle into a shelf slot: 0 → 0, 2 → 1, 4 → 2 ... */
  uint16_t slot = (uint16_t)((bearing - PAN_MIN) / PAN_STEP_DEG);
  if (slot >= PAN_SLOTS) {
    return;                                     /* off the end: reject, don't corrupt memory */
  }

  if (calibrating) {
    /* Learning: write this reading into the current sweep's column. */
    if (sweep_counter >= BASELINE_SWEEPS) {
      return;
    }
    background[slot][sweep_counter] = distance;

  } else {
    /* Watching: is this reading a lot closer than the empty room?
       (Written as "distance + 40 < baseline" rather than "distance 
       baseline - 40", because both are unsigned: a baseline under 40
       would wrap around to a huge number.) */
    if (distance + MIN_DROP_CM < baseline[slot]) {
      /* Close: the run grows. */
      if (readings_count == 0) {                /* first reading of a new run */
        first_bearing = bearing;
        first_dist    = distance;
        min_dist      = distance;
      }
      last_bearing = bearing;                   /* always the most recent */
      last_dist    = distance;
      if (distance < min_dist) {
        min_dist = distance;                    /* keep the closest */
      }
      readings_count++;

    } else {
      /* Not close: whatever run was going just ended. */
      close_run();
    }
  }
}

/* Called by main.c when the head turns around at either end. */
void detect_sweep_end(void) {
  if (calibrating) {
    /* One more column filled. After the last one, squash each angle's
       three readings into one number and switch to watching. */
    sweep_counter++;
    if (sweep_counter == BASELINE_SWEEPS) {
      for (uint8_t i = 0; i < PAN_SLOTS; i++) {
        baseline[i] = median3(background[i][0], background[i][1], background[i][2]);
      }
      calibrating = false;
    }
  }
  close_run();
}

uint8_t detect_result(detection_t detection_array[], uint8_t max_slots){
  uint8_t to_copy;

  to_copy=object_count;
  if(to_copy>max_slots){ to_copy=max_slots;}

  for(uint8_t i=0;i<to_copy;i++){
    detection_array[i]=objects[i];
  }
  object_count=0;

  return to_copy;
}