#ifndef SIGNALS_H
#define SIGNALS_H

#include "arm_math.h"

#define sig1_len 301
#define sig2_len 320
#define sig3_len 29
#define sig_ecg_len 640
#define mixed_sig_len 1001
#define ECG_mudy_len 1800


extern float _5hz_signal[sig1_len];
extern float32_t inputSignal_f32_1kHz_15kHz[sig2_len];
extern float32_t  impulse_response[sig3_len];
extern float32_t _640_points_ecg_[sig_ecg_len];
extern float32_t mixed_sig[mixed_sig_len];
extern float32_t impulse_response_matLab[101];
extern float32_t ecg_mudy_sig[ECG_mudy_len];


#endif
