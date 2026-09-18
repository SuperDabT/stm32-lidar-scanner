#include "detect.h"
#include "servo.h"
#include <stdint.h>
#include <stdbool.h>

#define PAN_SLOTS (((int)(PAN_MAX-PAN_MIN) / (int)PAN_STEP_DEG) + 1)
#define BASELINE_SWEEPS 3

static uint16_t background[PAN_SLOTS][BASELINE_SWEEPS];
static uint8_t sweep_counter=0;

static bool calibrating=true

static uint16_t median3(uint16_t a,uint16_t b,uint16_t c){
    uint16_t temp_val=0;

    if(a>b){
        temp_val=a;
        a=b;
        b=temp_val;
    }
    if(a>c){
        temp_val=c;
        c=a;
        a=temp_val;
    }
    if(b>c){
        temp_val=c;
        c=b;
        b=temp_val;
    }
    return b;
}

void detect_calibrate(void){
    sweep_counter=0;
    calibrating=true;
}