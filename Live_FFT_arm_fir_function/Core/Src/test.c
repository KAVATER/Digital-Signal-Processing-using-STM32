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

// fifo.c
uint8_t rx_fifo_put(rx_DataType data)
{
    if (rx_fifo_count >= RXFIFO_Size) return RXFIFO_Fail;   // truly full
    *rx_put_pt = data;
    rx_put_pt++;
    if (rx_put_pt == &RX_FIFO[RXFIFO_Size]) rx_put_pt = &RX_FIFO[0];
    rx_fifo_count++;
    return RXFIFO_Done;
}

uint8_t rx_fifo_get(rx_DataType *datapt)
{
    if (rx_fifo_count == 0) return RXFIFO_Fail;   // truly empty
    *datapt = *rx_get_pt;
    rx_get_pt++;
    if (rx_get_pt == &RX_FIFO[RXFIFO_Size]) rx_get_pt = &RX_FIFO[0];
    rx_fifo_count--;
    return RXFIFO_Done;
}
