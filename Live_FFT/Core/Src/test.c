#define TX_CHUNK_SAMPLES  300
#define TX_BUF_SIZE       6144

static char tx_buf[2][TX_BUF_SIZE];
static volatile uint8_t tx_busy = 0;
static uint8_t tx_active = 0;

/* ===== In your main loop, after FFT ===== */
int sent = 0;
while (sent < received)
{
    /* Pick the idle buffer */
    uint8_t buf_idx = tx_active ^ 1;

    /* Wait until DMA is done with the other buffer */
    while (tx_busy && buf_idx == (tx_active ^ 1))
    {
        /* Or just break and skip this chunk if you prefer non-blocking */
    }

    /* Format one chunk */
    size_t len = 0;
    int chunk_end = sent + TX_CHUNK_SAMPLES;
    if (chunk_end > received) chunk_end = received;

    for (int i = sent; i < chunk_end; i++)
    {
        if (len + 32 > TX_BUF_SIZE) break;
        len += snprintf(tx_buf[buf_idx] + len,
                        TX_BUF_SIZE - len,
                        "%u,%.6f\r\n",
                        (unsigned)adc_buff[i],
                        FFT_Buff_Out_aligned[i]);
    }

    /* Fire DMA */
    if (!tx_busy)
    {
        tx_active = buf_idx;
        tx_busy = 1;
        HAL_UART_Transmit_DMA(&huart2,
                              (uint8_t *)tx_buf[tx_active],
                              len);
    }

    sent += TX_CHUNK_SAMPLES;
}

/* ===== DMA complete callback ===== */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        tx_busy = 0;
    }
}
