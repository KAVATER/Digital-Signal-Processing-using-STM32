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
#define moving_avg_pts 11
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define adc_buff_len 150
uint16_t adc_buff[adc_buff_len];

extern float _5hz_signal[sig1_len];

extern float32_t inputSignal_f32_1kHz_15kHz[sig2_len];
 float32_t inputSignal_f32_1kHz_15kHz_out[sig2_len];

extern float32_t  impulse_response[sig3_len];
extern float32_t  _640_points_ecg_[sig_ecg_len];



void convolution(float32_t* sig_arr, float32_t* destination_array,float32_t* imp_resp,
		         uint32_t sig_src_len, uint32_t imp_resp_len);


void serialplot_outputSig_convolved(float32_t* destination_array);

float in_sig_sample;
float imp_rsp_sample;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void cal_running_sum(float32_t *sig_arr, float32_t *sig_dest_arr, uint32_t sig_len);
void plot_running_sum(float32_t* destination_array);

void cal_sig_DFT(float32_t *sig_arr, float32_t* sig_rex_arr,
		         float32_t *sig_imx_arr, uint32_t sig_len);
void plot_cal_sig_DFT(float32_t *arr1,uint32_t sig_len);
void get_dft_output_mag(uint32_t sig_len);

void plot_cal_sig_DFT_both(float32_t *real_arr,
        float32_t *imag_arr,
        uint32_t sig_len);

void cal_sig_IDFT(float32_t *idft_out_arr, float32_t* sig_rex_arr,
		         float32_t *sig_imx_arr, uint32_t idft_len);


void plot_signal(float32_t *arr,uint32_t sign_len);

float REX[sig_ecg_len/2];
float IMX[sig_ecg_len/2];
float32_t magnitude[sig_ecg_len/2];
void  mag_plot(float32_t *arr, uint32_t sig_len);
float32_t idft_out_arr[sig_ecg_len];

extern float32_t impulse_response_matLab[101];
extern float32_t mixed_sig[mixed_sig_len];
float32_t output_sig_matLab[mixed_sig_len+101-1];

void moving_avg(float32_t* sig_src_arr, float32_t* sig_out_arr, uint32_t signal_len,
		       uint32_t filter_pts);
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
//int _write(int fd, char *ptr, int len)
//{
//	HAL_StatusTypeDef hstatus;
//	if(fd==1 || fd==2)
//	{
//		hstatus =  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
//		  return len;
//
//		  if(hstatus == HAL_OK)
//		  return len;
//
//		  else
//			  return -1;
//	}
//	return -1;
//}
int _write(int fd, char *ptr, int len)
{
	if(fd==1 || fd==2)
	{
		if (HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY) == HAL_OK)
			return len;
		return -1;
	}
	return -1;
}
volatile uint8_t flag = 0;
uint16_t sample = 0;
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

//  moving_avg(inputSignal_f32_1kHz_15kHz, inputSignal_f32_1kHz_15kHz_out, sig2_len, moving_avg_pts);
//  plot_signal( inputSignal_f32_1kHz_15kHz_out,sig2_len );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1);
	  sample= HAL_ADC_GetValue(&hadc1);
	  printf("%u\r\n",sample);
	  HAL_Delay(500);
  }
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
void plot_signal(float32_t *arr,uint32_t sig_len)
{
	for(int i=0; i<sig_len; i++)
		    {
		        long val = (long)(arr[i] * 100000);
		        long int_part  = val / 100000;
		        long frac_part = labs(val % 100000);

		        if (val < 0 && int_part == 0) {
		            printf("-%ld.%05ld\r\n", int_part, frac_part);
		        } else {
		            printf("%ld.%05ld\r\n", int_part, frac_part);
		        }

		        HAL_Delay(50);
		    }

}

void moving_avg(float32_t* sig_src_arr, float32_t* sig_out_arr, uint32_t signal_len,
		       uint32_t filter_pts)
{
	for(int i = floor(filter_pts/2); i<(signal_len - (filter_pts/2))-1; i++)
	{
		sig_out_arr[i] = 0;
		for(int j = -(floor(filter_pts/2)); j<floor(filter_pts/2); j++)
		{
			sig_out_arr[i] = sig_out_arr[i] + sig_src_arr[i+j];
		}
		sig_out_arr[i] = sig_out_arr[i]/filter_pts;
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
