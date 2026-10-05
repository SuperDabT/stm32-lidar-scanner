#include "track.h"
#include "detect.h"
#include <stdint.h>

#define PERSON_MIN_WIDTH_CM 20 /* Not 25: a person walking against the sweep
                                   direction looks narrower (23-24 cm seen in
                                   person_walking_slow.csv). Still well above
                                   noise blobs like chair legs (~5-15 cm). */
#define PERSON_MAX_WIDTH_CM 80
#define CONFIRM_WINDOW 4

#define HISTORY_MASK 0x0F  /* keep the 4 newest looks (bits 0-3) */

static track_t target;

static void push_look(bool seen){
    target.look_history<<=1;
    target.look_history|=seen;
    target.look_history&=HISTORY_MASK;
}

static uint8_t count_hits(void){
    uint8_t hits=0;;
    uint8_t mask=0;
    for(int i=0;i<4;i++){
        mask=1<<i;
        if((target.look_history&mask)>0){
            hits++;
        }
    }
    return hits;
}



track_t track_update(detection_t detection_array[],uint8_t sus_objects){
    switch (target.target_state) {
        case STATE_SURVEY:
        for(uint8_t i=0;i<sus_objects;i++){
            if(detection_array[i].width>=PERSON_MIN_WIDTH_CM && detection_array[i].width<=PERSON_MAX_WIDTH_CM){
                to_xy_t point;
                point=convert_xy(detection_array[i].bearing, detection_array[i].distance);

                target.first_seen.x=point.x;
                target.first_seen.y=point.y;

                target.position.x=point.x;
                target.position.y=point.y;

                target.seen_at_ms=HAL_GetTick();

                target.target_state=STATE_CONFIRMING;
                break;

            }
        }
        
        break;
    case STATE_CONFIRMING:
    //TODO

    break;
    }
    return target;
}
