#include "detect.h"
#include "servo.h"
#include <stdint.h>
#include <stdbool.h>
#include <sys/_intsup.h>

#define PAN_SLOTS (((int)(PAN_MAX-PAN_MIN) / (int)PAN_STEP_DEG) + 1)
#define BASELINE_SWEEPS 3

static uint16_t background[PAN_SLOTS][BASELINE_SWEEPS];
static uint16_t baseline[PAN_SLOTS];
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

void detect_feed(uint16_t bearing, uint16_t distance){
    uint16_t slot=(uint16_t)((bearing-PAN_MIN)/PAN_STEP_DEG);
    if(slot>=PAN_SLOTS){
    return;
    }
    
    if(calibrating){
    if(sweep_counter>=BASELINE_SWEEPS){
        return;
    }
        background[slot][sweep_counter]=distance;
    }
}

void detect_sweep_end(void){
    if(calibrating){
        sweep_counter++;
    if(sweep_counter==BASELINE_SWEEPS){
    for(uint8_t i=0;i<PAN_SLOTS;i++){
        baseline[i]=median3(background[i][0], background[i][1], background[i][2]);
    
    }
    calibrating=false;
    }
    }
}