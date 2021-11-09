#ifndef __TF_H
#define __TF_H

#include <stdint.h>
typedef struct 
{
    float *deno_coeff;
    float *nume_coeff;
    uint8_t order;
    float *state_x;
    float *state_y;
}tf_t;

#define SERVO_DYN_TF_ORDER 2
#define SERVO_DYN_TF_DIMENSION (SERVO_DYN_TF_ORDER + 1)

#endif
