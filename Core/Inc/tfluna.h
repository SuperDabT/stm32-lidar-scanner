#ifndef TFLUNA_H
#define TFLUNA_H

#include <stdint.h>
#include <stdbool.h>


uint16_t tfluna_distance(void);



bool tfluna_valid(void);

void tfluna_init(void);

float tfluna_temperature(void);


#endif