#ifndef TFLUNA_H
#define TFLUNA_H

#include <stdint.h>
#include <stdbool.h>

typedef struct{

    bool valid;
    uint16_t distance;



}tfluna_reading_t;


tfluna_reading_t tfluna_read(void);

void tfluna_init(void);

float tfluna_temperature(void);


#endif