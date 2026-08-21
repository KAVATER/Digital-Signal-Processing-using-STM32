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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stm32f407xx.h"
#include "signals.h"
#include "stdio.h"
#include "arm_math.h"
#include <math.h>
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern float _5hz_signal[sig1_len];
extern float32_t inputSignal_f32_1kHz_15kHz[sig2_len];
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


void plot_signal(float32_t *arr,uint32_t sign_len,uint32_t sign2_len);

float REX[sig_ecg_len/2];
float IMX[sig_ecg_len/2];
float32_t magnitude[sig_ecg_len/2];
void  mag_plot(float32_t *arr, uint32_t sig_len);
float32_t idft_out_arr[sig_ecg_len];

extern float32_t impulse_response_matLab[101];
extern float32_t mixed_sig[mixed_sig_len];
float32_t output_sig_matLab[mixed_sig_len+101-1];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int fd, char *ptr, int len)
{
	HAL_StatusTypeDef hstatus;
	if(fd==1 || fd==2)
	{
		hstatus =  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
		  return len;

		  if(hstatus == HAL_OK)
		  return len;

		  else
			  return -1;
	}
	return -1;
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
  /* USER CODE BEGIN 2 */

//	plot_ecg(_640_points_ecg_,sig_ecg_len);
//
//	cal_sig_DFT(_640_points_ecg_,REX ,
//			IMX, sig_ecg_len);
//
//	cal_sig_IDFT(idft_out_arr,REX,IMX,sig_ecg_len);
//
//		plot_ecg(idft_out_arr, sig_ecg_len);

  arm_conv_f32(impulse_response_matLab , 101,mixed_sig, mixed_sig_len,output_sig_matLab);

  plot_signal(output_sig_matLab,mixed_sig_len,101);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void get_dft_output_mag(uint32_t sig_len)
{
	for(int k = 0 ; k< (sig_len/2); k++)
	{
		magnitude[k] = sqrtf(
		    REX[k] * REX[k] +
		    IMX[k] * IMX[k]
		);
	}

}

void cal_sig_IDFT(float32_t *idft_out_arr, float32_t* sig_rex_arr,
		         float32_t *sig_imx_arr, uint32_t idft_len)
{
	/*Normalize amplitude*/
  for(int i = 1; i<idft_len/2; i++)
  {
	 sig_rex_arr[i] = sig_rex_arr[i]/(idft_len/2);
	 sig_imx_arr[i] = -sig_rex_arr[i]/(idft_len/2);
  }

  //Index 0
  sig_rex_arr[0] = sig_rex_arr[0]/(idft_len);
  sig_imx_arr[0] = -sig_rex_arr[0]/(idft_len);

  //Index 1
  sig_rex_arr[idft_len/2] = sig_rex_arr[idft_len/2]/(idft_len);
  sig_imx_arr[idft_len/2] = -sig_imx_arr[idft_len/2]/(idft_len);

  for(int j = 1; j<idft_len; j++)
  {
	  idft_out_arr[j] = 0;
  }

  for(int k = 0; k<idft_len/2;k++)
  {
	  for(int i = 0; i <  idft_len;i ++)
	  {
		  idft_out_arr[i] = idft_out_arr[i] + sig_rex_arr[k]*cos(2*PI*k*i/idft_len);
		  idft_out_arr[i] = idft_out_arr[i] + sig_rex_arr[k]*sin(2*PI*k*i/idft_len);
	  }
  }
}

void cal_sig_DFT(float32_t *sig_arr, float32_t* sig_rex_arr,
		         float32_t *sig_imx_arr, uint32_t sig_len)
{

	uint32_t i,j,k = 0;
  for( i = 0; i< sig_len; i++ )
  {
	  sig_rex_arr[i] = 0;
	  sig_imx_arr[i] = 0;
  }

  /*compute DFT*/
  for(k = 0; k <sig_len/2; k++)
  {
	  for(j = 0; j<sig_len; j++)
	  {
		  sig_rex_arr[k]  = sig_rex_arr[k] + sig_arr[j]*cos(2*PI*k*j/sig_len);//real part
		  sig_imx_arr[k]  = sig_imx_arr[k] + sig_arr[j]*sin(2*PI*k*j/sig_len);//imaginary part
	  }
  }

}
void plot_signal(float32_t *arr,uint32_t sig_len, uint32_t sign2_len)
{
	for(int i=0; i<sig_len+sig2_len-1; i++)
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
void  mag_plot(float32_t *arr, uint32_t sig_len)
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

			       // sudo_delay(9000);
			    }
}
void plot_cal_sig_DFT(float32_t *arr,uint32_t sig_len)
{
	sig_len = sig_len/2;

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

		  //      sudo_delay(9000);
		    }
}

void plot_cal_sig_DFT_both(float32_t *real_arr,
                      float32_t *imag_arr,
                      uint32_t sig_len)
{
    sig_len = sig_len / 2;

    for (int i = 0; i < sig_len; i++)
    {
        /* Real part */
        long real_val = (long)(real_arr[i] * 100000);
        long real_int_part = real_val / 100000;
        long real_frac_part = labs(real_val % 100000);

        /* Imaginary part */
        long imag_val = (long)(imag_arr[i] * 100000);
        long imag_int_part = imag_val / 100000;
        long imag_frac_part = labs(imag_val % 100000);

        printf("Re: ");
        if (real_val < 0 && real_int_part == 0)
            printf("-%ld.%05ld", real_int_part, real_frac_part);
        else
            printf("%ld.%05ld", real_int_part, real_frac_part);

        printf("  Im: ");

        if (imag_val < 0 && imag_int_part == 0)
            printf("-%ld.%05ld\r\n", imag_int_part, imag_frac_part);
        else
            printf("%ld.%05ld\r\n", imag_int_part, imag_frac_part);

        //sudo_delay(9000);
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
