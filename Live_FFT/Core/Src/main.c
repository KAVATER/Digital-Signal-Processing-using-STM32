/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stm32f407xx.h"
#include "signals.h"
#include "stdio.h"
#include "arm_math.h"
#include <math.h>
#include "stdlib.h"
#include "fifo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define FILTER_Taps fir_len

#define input_sig_len live_ecg_len
#define filter_len fir_len

extern const float32_t  _640_points_ecg_[sig_ecg_len];
extern const float32_t ecg_mudy_sig[ECG_mudy_len];
extern const float32_t fir_filter[fir_len];

extern volatile uint8_t sample_tick;

/* ---- Compile-time "round up to next power of two" -------------------------
 * Classic bit-smearing trick: OR each bit down into all bits below it, then
 * add 1. Works purely with constant arithmetic, so the result can be used
 * as an array size. */
#define _B2(x)   ( (x)      | ((x) >> 1)  )
#define _B4(x)   ( _B2(x)   | (_B2(x) >> 2)  )
#define _B8(x)   ( _B4(x)   | (_B4(x) >> 4)  )
#define _B16(x)  ( _B8(x)   | (_B8(x) >> 8)  )
#define _B32(x)  ( _B16(x)  | (_B16(x) >> 16) )
#define NEXT_POW2(x)  (_B32((x) - 1) + 1)

/* ===== OR ===== */

//uint32_t next_power_of_2(uint32_t x)
//{
//    if (x == 0) return 1;
//
//    x = x - 1;          // step 1: back off by one (so exact powers of 2 stay unchanged)
//
//    // step 2: "smear" the highest set bit downward into every bit below it,
//    // by repeatedly OR-ing the number with a right-shifted copy of itself
//    x = x | (x >> 1);
//    x = x | (x >> 2);
//    x = x | (x >> 4);
//    x = x | (x >> 8);
//    x = x | (x >> 16);
//    // at this point x is all 1-bits, e.g. 0b00011111 (31)
//
//    x = x + 1;           // step 3: turn "all 1s" into the actual power of two
//
//    return x;
//}

/* Length needed for a full ("linear") convolution of the ECG signal with the
 * FIR filter: len(x) + len(h) - 1.
 *
 * NOTE: renamed from the original "L" to "LINEAR_CONV_LEN". A macro named
 * "L" would have been substituted INSIDE the prototype
 * "uint32_t next_power_of_2(uint32_t L)" below, mangling the parameter name -
 * a subtle bug waiting to happen even after fixing the file-scope issue. */
#define LINEAR_CONV_LEN  (input_sig_len + filter_len - 1)

/* Smallest power-of-two FFT length that can hold the full linear convolution
 * without circular-convolution wrap-around (this is a classic
 * overlap-save / fast-convolution-via-FFT length requirement). Computed at
 * compile time now - no more runtime "fft_len" needed for sizing. */
#define FFT_BUFFER_SIZE  NEXT_POW2(LINEAR_CONV_LEN)

/* arm_rfft_fast_f32 (CMSIS-DSP, f32 variant) only supports power-of-two
 * lengths from 32 to 4096. Catch an unsupported combination of signal /
 * filter lengths at BUILD time instead of arm_rfft_fast_init_f32() silently
 * returning an error status at runtime. */
#if (FFT_BUFFER_SIZE < 32) || (FFT_BUFFER_SIZE > 4096)
#error "FFT_BUFFER_SIZE is outside the range supported by arm_rfft_fast_f32 (32-4096). Check input_sig_len/filter_len in signals.h."
#endif

float32_t padded_filter[FFT_BUFFER_SIZE];
float32_t padded_filter2[FFT_BUFFER_SIZE];
float32_t FFT_Buff_In[FFT_BUFFER_SIZE];
float32_t FFT_Buff_In2[FFT_BUFFER_SIZE];
float32_t FFT_Buff_Out[FFT_BUFFER_SIZE];
float32_t FFT_Buff_Out_original[LINEAR_CONV_LEN];
float32_t conv_out[LINEAR_CONV_LEN];
float32_t conv_out_aligned[input_sig_len];   /* group-delay-compensated, same length as input */

float32_t fft_time_result[FFT_BUFFER_SIZE];

#define GROUP_DELAY  ((filter_len - 1) / 2)

/* Time-aligned filtered output: FFT_Buff_Out_original[n + GROUP_DELAY] is
 * what actually corresponds to input sample n, once the filter's inherent
 * delay is removed. Same length as the input signal, so it plots directly
 * against ecg_mudy_sig sample-for-sample with no offset. */
float32_t FFT_Buff_Out_aligned[input_sig_len];
volatile float32_t inspect_var;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void plot_signal(float32_t *arr,uint32_t sign_len);
arm_rfft_fast_instance_f32 fftHandler;

static uint16_t format_sample(float32_t sample, char *dst, uint16_t dst_size);
void plot_signal_2(float32_t *arr1, uint32_t len1, float32_t *arr2, uint32_t len2);
 void plot_signal_3(float32_t *arr1, uint32_t len1, float32_t *arr2, uint32_t len2,float32_t *arr3, uint32_t len3);


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ===== Fifo variables ===== */
volatile uint8_t  flag = 0;
uint32_t sample_count = 0;
volatile uint32_t fifo_overflow_count = 0;   /* FIFO was full when a sample arrived */
volatile uint32_t adc_timeout_count   = 0;   /* ADC didn't finish converting in time */

uint16_t val = 0; //rx_DataType
rx_DataType rx_data;
#define adc_buff_len 1500
rx_DataType adc_buff[adc_buff_len];   /* same type as the FIFO holds */

uint32_t adc_ready = 0;
char bf[50];
/* ===== END ===== */

void reduce_to_original_len(float32_t *output_buff)
{
    for (uint32_t i = 0; i < LINEAR_CONV_LEN; i++)
	{
        FFT_Buff_Out_original[i] = output_buff[i];
	}
}
/* ===== Systick CallBack ===== */
//void HAL_SYSTICK_Callback(void)
//{
//	if(!adc_ready)
//	{
//		return;
//	}
//    if (HAL_ADC_Start(&hadc1) != HAL_OK)
//    {
//        return;
//    }
//
//    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
//    {
//        val = (rx_DataType)HAL_ADC_GetValue(&hadc1);
//
//        if (rx_fifo_put(val) == RXFIFO_Done)
//        {
//            sample_count++;
//            if (sample_count >= adc_buff_len)
//            {
//                flag = 1;
//                sample_count = 0;
//            }
//        }
//        else
//        {
//            fifo_overflow_count++;   /* main loop is draining too slowly */
//        }
//    }
//    else
//    {
//        adc_timeout_count++;         /* diagnostic only — should stay 0 */
//    }
//}
/* ===== END ===== */

char tx_buf[8192];
uint16_t tx_len;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  rx_fifo_init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //HAL_ADC_Start_IT(&hadc1);


      adc_ready = 1;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(sample_tick)
	  	  {
	  		  sample_tick = 0;

	  	        if (HAL_ADC_Start(&hadc1) == HAL_OK &&
	  	            HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
	  	        {
	  	            val = (rx_DataType)HAL_ADC_GetValue(&hadc1);
	  	            if (rx_fifo_put(val) == RXFIFO_Done)
	  	            {
	  	                sample_count++;
	  	                if (sample_count >= adc_buff_len)
	  	                {
	  	                    flag = 1;
	  	                    sample_count = 0;
	  	                }
	  	            }
	  	            else { fifo_overflow_count++; }
	  	        }
	  	        else { adc_timeout_count++; }

	  	  }
	  /* ===== ADC buffer fill ===== */

	  if (flag == 1)
	 	      {
	 	          int received = 0;

	 	          for (int i = 0; i < adc_buff_len; i++)
	 	          {
	 	              if (rx_fifo_get(&rx_data) == RXFIFO_Done)
	 	              {
	 	                  adc_buff[i] = rx_data;
	 	                  received++;
	 	              }
	 	              else
	 	              {
	 	                  break;
	 	              }
	 	          }

		  /* ===== copy input signal, then zero-pad the rest of the FFT buffer ===== */
		  for (uint32_t i = 0; i < input_sig_len; i++) {
		      FFT_Buff_In[i] = adc_buff[i];
		  }

		  for (uint32_t j = input_sig_len; j < FFT_BUFFER_SIZE; j++)
		  {
			  FFT_Buff_In[j] = 0;
		  }

		  /* ===== copy filter coefficients, then zero-pad the rest ===== */
		  for (uint32_t i = 0; i < filter_len; i++) {

		        padded_filter[i] = fir_filter[i];
		    }
		  for (uint32_t i = filter_len; i < FFT_BUFFER_SIZE; i++)
		  {
			  padded_filter[i] = 0;
		  }

		        /* ===== Initializing fft  any length outside {32,64,...,4096} ===== */

		  if (arm_rfft_fast_init_f32(&fftHandler, FFT_BUFFER_SIZE) != ARM_MATH_SUCCESS)
		  {
		      Error_Handler();
		  }

		              /* FFT of input buffer (in-place: arm_rfft_fast_f32 supports p == pOut) */
		    arm_rfft_fast_f32(&fftHandler,FFT_Buff_In,FFT_Buff_In2, 0);

		           /*FFT of padded_filter*/

		    arm_rfft_fast_f32(&fftHandler,padded_filter,padded_filter2,0);

		      /* ===== Multiply the two spectra (frequency-domain = fast convolution) =====
		       * arm_rfft_fast_f32's packed output format:
		       *   index 0        -> DC bin, purely real
		       *   index 1        -> Nyquist bin, purely real
		       *   index 2..N-1    -> complex (Re, Im) pairs for the remaining bins
		       */
		      // DC bin (index 0): pure real

		      FFT_Buff_Out[0] = FFT_Buff_In2[0] * padded_filter2[0];

		      // Nyquist bin (index 1): pure real

		      FFT_Buff_Out[1] = FFT_Buff_In2[1] * padded_filter2[1];

		      // Remaining bins: complex (Re, Im) pairs starting at index 2
		      for (uint32_t i = 2; i < FFT_BUFFER_SIZE; i += 2)
		      {
		          float32_t a = FFT_Buff_In2[i];        // Re{X[k]}
		          float32_t b = FFT_Buff_In2[i + 1];    // Im{X[k]}
		          float32_t c = padded_filter2[i];     // Re{H[k]}
		          float32_t d = padded_filter2[i + 1]; // Im{H[k]}

		         /* ===== (a + jb) * (c + jd) = (ac - bd) + j(ad + bc) ===== */
		          FFT_Buff_Out[i]     = a * c - b * d;   // Re{Y[k]}
		          FFT_Buff_Out[i + 1] = a * d + b * c;   // Im{Y[k]}
		      }

		      /* ===== Inverse FFT: back to the time domain ===== */
		      arm_rfft_fast_f32(&fftHandler, FFT_Buff_Out, fft_time_result, 1);

		      /* ===== Trim the zero-padded FFT result down to the true linear-
		       * convolution length (input_len + filter_len - 1) ===== */
		      reduce_to_original_len(fft_time_result);

		      /* ===== compensate fir group delay ===== */

		      for (uint32_t n = 0; n < input_sig_len; n++)
		      {
		          FFT_Buff_Out_aligned[n] = FFT_Buff_Out_original[n + GROUP_DELAY];
		      }

//		      for(uint32_t k = 0; k<input_sig_len; k++)
//		      {
//		    	  inspect_var = FFT_Buff_Out_aligned[k];
//		      }

		      /* ===== Convolution to cross check the fft result ===== */
		      // arm_conv_f32(ecg_mudy_sig, input_sig_len, fir_filter, filter_len, conv_out);

		       /* Same group-delay compensation as the FFT path, so both traces are
		        * directly comparable: same length (input_sig_len), same time alignment. */
		    //   for (uint32_t n = 0; n < input_sig_len; n++)
		    //   {
		    //       conv_out_aligned[n] = conv_out[n + GROUP_DELAY];
		    //   }
		          /* ===== Plotting ===== */

		       //   plot_signal_2((float32_t*)adc_buff,adc_buff_len , FFT_Buff_Out_aligned, input_sig_len);
		         // plot_signal_3(ecg_mudy_sig, input_sig_len, conv_out_aligned,input_sig_len,FFT_Buff_Out_aligned, input_sig_len );

//			      for (int i = 0; i < received; i++)
//		      	 	          {
//		      	 	              sprintf(bf, "%d\r\n", adc_buff[i]);
//		      	 	              HAL_UART_Transmit(&huart2, (uint8_t *)bf, strlen(bf), HAL_MAX_DELAY);
//		      	 	          }

		      for (int i = 0; i < received; i++)
		      {
		          int len = snprintf(bf, sizeof(bf),
		                             "%u,%.6f\r\n",
		                             (unsigned)adc_buff[i],
		                             FFT_Buff_Out_aligned[i]);
		          HAL_UART_Transmit(&huart2, (uint8_t *)bf, len, HAL_MAX_DELAY);
		      }

		      /* ===== Format the ENTIRE block first ===== */
//		      tx_len = 0;
//		      for (int i = 0; i < received; i++)
//		      {
//		          tx_len += snprintf(tx_buf + tx_len,
//		                             sizeof(tx_buf) - tx_len,
//		                             "%u,%.6f\r\n",
//		                             (unsigned)adc_buff[i],
//		                             FFT_Buff_Out_aligned[i]);
//		      }
//
//		      /* ===== Then fire DMA once, outside the loop ===== */
//		      if (huart2.gState == HAL_UART_STATE_READY)
//		      {
//		          HAL_UART_Transmit_DMA(&huart2, (uint8_t *)tx_buf, tx_len);
//		      }
//		      else
//		      {
//
//		      }

		      /* ===== END of Plotting ===== */
	 	          flag = 0;
	 	      }
	 	    }

	  /* ===== ADC Testing Code ===== */

//	  HAL_ADC_Start(&hadc1);
//	  HAL_ADC_PollForConversion(&hadc1, 1);
//	  val = HAL_ADC_GetValue(&hadc1);
//      sprintf(bf, "%d\r\n", val);
//      HAL_UART_Transmit(&huart2, (uint8_t *)bf, strlen(bf), HAL_MAX_DELAY);

	 /* ===== END ===== */


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void plot_signal(float32_t *arr, uint32_t sig_len)
{
    char tx_buf[32];
    uint16_t len;

   // sig_len = sig_len / 2;

    for (uint32_t i = 0; i < sig_len; i++)
    {
        long val = (long)(arr[i] * 100000.0f);
        long int_part  = val / 100000;
        long frac_part = labs(val % 100000);

        if (val < 0 && int_part == 0) {
            len = snprintf(tx_buf, sizeof(tx_buf), "-%ld.%05ld\r\n", int_part, frac_part);
        } else {
            len = snprintf(tx_buf, sizeof(tx_buf), "%ld.%05ld\r\n", int_part, frac_part);
        }

        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buf, len, HAL_MAX_DELAY);
        HAL_Delay(50);
    }
}
static uint16_t format_sample(float32_t sample, char *dst, uint16_t dst_size)
{
    long val = (long)(sample * 100000.0f);
    long int_part  = val / 100000;
    long frac_part = labs(val % 100000);

    if (val < 0 && int_part == 0) {
        return snprintf(dst, dst_size, "-%ld.%05ld", int_part, frac_part);
    } else {
        return snprintf(dst, dst_size, "%ld.%05ld", int_part, frac_part);
    }
}
void plot_signal_2(float32_t *arr1, uint32_t len1, float32_t *arr2, uint32_t len2)
{
    char tx_buf[40];
    char val_buf[16];
    uint16_t len;
    uint32_t max_len = (len1 > len2) ? len1 : len2;

    for (uint32_t i = 0; i < max_len; i++)
    {
        if (i < len1) {
            format_sample(arr1[i], val_buf, sizeof(val_buf));
        } else {
            val_buf[0] = '\0';   // ran out of samples for signal 1 - leave blank
        }
        len = snprintf(tx_buf, sizeof(tx_buf), "%s,", val_buf);

        if (i < len2) {
            format_sample(arr2[i], val_buf, sizeof(val_buf));
        } else {
            val_buf[0] = '\0';   // ran out of samples for signal 2 - leave blank
        }
        len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "%s\r\n", val_buf);

        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buf, len, HAL_MAX_DELAY);
        HAL_Delay(50);
    }
}
void plot_signal_3(float32_t *arr1, uint32_t len1,
                  float32_t *arr2, uint32_t len2,
                  float32_t *arr3, uint32_t len3)
{
    char tx_buf[80];
    char val_buf[24];
    uint16_t len;

    uint32_t max_len = len1;

    if (len2 > max_len)
        max_len = len2;

    if (len3 > max_len)
        max_len = len3;

    for (uint32_t i = 0; i < max_len; i++)
    {
        len = 0;

        /* Signal 1 */
        if (i < len1)
        {
            format_sample(arr1[i], val_buf, sizeof(val_buf));
            len += snprintf(tx_buf + len,
                            sizeof(tx_buf) - len,
                            "%s,", val_buf);
        }

        /* Signal 2 */
        if (i < len2)
        {
            format_sample(arr2[i], val_buf, sizeof(val_buf));
            len += snprintf(tx_buf + len,
                            sizeof(tx_buf) - len,
                            "%s,", val_buf);
        }

        /* Signal 3 */
        if (i < len3)
        {
            format_sample(arr3[i], val_buf, sizeof(val_buf));
            len += snprintf(tx_buf + len,
                            sizeof(tx_buf) - len,
                            "%s", val_buf);
        }

        /* New line */
        len += snprintf(tx_buf + len,
                        sizeof(tx_buf) - len,
                        "\r\n");

        HAL_UART_Transmit(&huart2,
                          (uint8_t *)tx_buf,
                          len,
                          HAL_MAX_DELAY);

        HAL_Delay(50);
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
