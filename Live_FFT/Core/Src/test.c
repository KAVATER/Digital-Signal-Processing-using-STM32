#define NUM_TAPS   fir_len        // 4240 — keep your MATLAB filter as-is
#define BLOCK_SIZE live_ecg_len   // 500

arm_fir_instance_f32 firFilter;
float32_t firState[NUM_TAPS + BLOCK_SIZE - 1];   /* persists across blocks */

/* once, before while(1) */
arm_fir_init_f32(&firFilter, NUM_TAPS, (float32_t *)fir_filter, firState, BLOCK_SIZE);

/* per live block, inside if (flag == 1) */
float32_t block_in[BLOCK_SIZE], block_out[BLOCK_SIZE];
for (uint32_t i = 0; i < BLOCK_SIZE; i++) block_in[i] = (float32_t)adc_buff[i];
arm_fir_f32(&firFilter, block_in, block_out, BLOCK_SIZE);