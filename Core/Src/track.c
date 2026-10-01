#include "track.h"
#include "detect.h"
#include <stdint.h>

#define PERSON_MIN_WIDTH_CM 20
#define PERSON_MAX_WIDTH_CM 80

static track_t target;





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
