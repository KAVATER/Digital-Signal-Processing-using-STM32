#ifndef SIGNALS_H
#define SIGNALS_H

#include "arm_math.h"

#define sig1_len 301
#define sig2_len 320
#define sig3_len 29

extern float _5hz_signal[sig1_len];
extern float32_t inputSignal_f32_1kHz_15kHz[sig2_len];
extern float32_t  impulse_response[sig3_len];

#endif
