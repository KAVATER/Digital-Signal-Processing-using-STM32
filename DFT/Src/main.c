#include "stm32f407xx.h"
#include "signals.h"
#include "uart.h"
#include "stdio.h"
#include "arm_math.h"
#include <math.h>
#include "stdlib.h"
#include "systick.h"

#define GPIOAEN  (1U<<0)
#define PIN5    (1U<<5)
#define LED_PIN  PIN5

//#define ITM_PORT0 (*(volatile uint32_t *)0xE0000000)

extern float _5hz_signal[sig1_len];
extern float32_t inputSignal_f32_1kHz_15kHz[sig2_len];
extern float32_t  impulse_response[sig3_len];
extern float32_t  _640_points_ecg_[sig_ecg_len];


void convolution(float32_t* sig_arr, float32_t* destination_array,float32_t* imp_resp,
		         uint32_t sig_src_len, uint32_t imp_resp_len);



//float32_t destination_arr2[sig2_len + sig3_len];
//float32_t destination_arr3[sig2_len + sig3_len-1];

void serialplot_outputSig_convolved(float32_t* destination_array);

float in_sig_sample;
float imp_rsp_sample;

static void fpu_enable(void);

//float32_t output_arr[sig2_len];

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

static void sudo_delay(uint16_t dly);
void plot_ecg(float32_t *arr,uint32_t sign_len);

float REX[sig_ecg_len/2];
float IMX[sig_ecg_len/2];
float32_t magnitude[sig_ecg_len/2];
void  mag_plot(float32_t *arr, uint32_t sig_len);
float32_t idft_out_arr[sig_ecg_len];

int main ()
{
   fpu_enable();

	/* Initilaize the uart*/
	uart2_tx_init();

	plot_ecg(_640_points_ecg_,sig_ecg_len);

	cal_sig_DFT(_640_points_ecg_,REX ,
			IMX, sig_ecg_len);

//	get_dft_output_mag(sig_ecg_len);
//
//	mag_plot(magnitude,sig_ecg_len/2);

	cal_sig_IDFT(idft_out_arr,REX,IMX,sig_ecg_len);

	plot_ecg(idft_out_arr, sig_ecg_len);

	while(1)
	{
	}
}

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
void plot_ecg(float32_t *arr,uint32_t sig_len)
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

		        //sudo_delay(9000);
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

        sudo_delay(9000);
    }
}

static void sudo_delay(uint16_t dly)
{
	for(volatile int j = 0; j<dly; j++){}
}

static void fpu_enable(void)
{
	/*Enable the floating point unit*/
	SCB->CPACR |=((3UL << 10*2) | (3UL << 11*2)); // arm cortex m device generic user guide
	// or

//	SCB->CPACR |= (1U<<20);
//	SCB->CPACR |= (1U<<21);
//	SCB->CPACR |= (1U<<22);
//	SCB->CPACR |= (1U<<23);
}
