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
#include "fifo.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
volatile uint8_t  flag = 0;
uint32_t sample_count = 0;
volatile uint32_t fifo_overflow_count = 0;   /* FIFO was full when a sample arrived */
volatile uint32_t adc_timeout_count   = 0;   /* ADC didn't finish converting in time */

rx_DataType val = 0;
rx_DataType rx_data;

/* USER CODE BEGIN PTD */
#define adc_buff_len 500
rx_DataType adc_buff[adc_buff_len];   /* same type as the FIFO holds */
/* USER CODE END PTD */
uint32_t adc_ready = 0;
void HAL_SYSTICK_Callback(void)
{
	if(!adc_ready)
	{
		return;
	}
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
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
        else
        {
            fifo_overflow_count++;   /* main loop is draining too slowly */
        }
    }
    else
    {
        adc_timeout_count++;         /* diagnostic only — should stay 0 */
    }
}
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
  adc_ready = 1;

  /* USER CODE BEGIN 2 */

//  moving_avg(inputSignal_f32_1kHz_15kHz, inputSignal_f32_1kHz_15kHz_out, sig2_len, moving_avg_pts);
//  plot_signal( inputSignal_f32_1kHz_15kHz_out,sig2_len );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /*Initializing fifo*/
  rx_fifo_init();
  char bf[50];
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

	          for (int i = 0; i < received; i++)
	          {
	              sprintf(bf, "%d\r\n", adc_buff[i]);
	              HAL_UART_Transmit(&huart2, (uint8_t *)bf, strlen(bf), HAL_MAX_DELAY);
	          }

	          flag = 0;
	      }
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
