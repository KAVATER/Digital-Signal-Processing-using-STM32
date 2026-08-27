/* Inverse FFT: do NOT use in-place for the inverse */
arm_rfft_fast_f32(&fftHandler, FFT_Buff_Out, FFT_Buff_In, 1);

/* Now the time-domain result is in FFT_Buff_In */
reduce_to_original_len(FFT_Buff_In);

/* Compensate group delay using the new buffer */
for (uint32_t n = 0; n < input_sig_len; n++)
{
    FFT_Buff_Out_aligned[n] = FFT_Buff_Out_original[n + GROUP_DELAY];
}

