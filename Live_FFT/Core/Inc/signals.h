#ifndef SIGNALS_H
#define SIGNALS_H

#include "arm_math.h"

#define sig_ecg_len 640
#define ECG_mudy_len  608
#define fir_len  3172
#define live_ecg_len 500


 extern const float32_t _640_points_ecg_[sig_ecg_len];
 extern const float32_t fir_filter[fir_len];
 extern const float32_t ecg_mudy_sig[ECG_mudy_len];


#endif
