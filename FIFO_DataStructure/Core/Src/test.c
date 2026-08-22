/* USER CODE BEGIN Includes */
#include "fifo.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
rx_DataType rx_data;   /* single variable to hold one FIFO item */
/* USER CODE END PV */

int main(void)
{
    /* ... init code ... */
    rx_fifo_init();
    HAL_ADC_Start_IT(&hadc1);

    /* Fill FIFO with dummy samples */
    for (int i = 0; i < adc_buff_len; i++)
    {
        rx_fifo_put(sample);
    }

    /* Drain FIFO into adc_buff — ONE BY ONE */
    for (int i = 0; i < adc_buff_len; i++)
    {
        if (rx_fifo_get(&rx_data) == RXFIFO_Done)
        {
            adc_buff[i] = rx_data;   /* ← use rx_data, not sample! */
        }
    }

    while (1)
    {
        if (flag == 1)
        {
            for (int i = 0; i < adc_buff_len; i++)
            {
                /* Process adc_buff[i] here */
            }
            flag = 0;
        }
    }
}
