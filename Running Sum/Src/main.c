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

static void plot_impulse_response(void);
static void serial_plot_impulse_response(void);
void convolution(float32_t* sig_arr, float32_t* destination_array,float32_t* imp_resp,
		         uint32_t sig_src_len, uint32_t imp_resp_len);


static void plot_input_signal(void);
static void sudo_delay(uint16_t dly);
float32_t destination_arr2[sig2_len + sig3_len];
float32_t destination_arr3[sig2_len + sig3_len-1];

void serialplot_outputSig_convolved(float32_t* destination_array);

float in_sig_sample;
float imp_rsp_sample;

static void fpu_enable(void);
static void serial_plotter(void);

uint32_t g_before, g_after, g_time_taken;
float32_t g_seconds, g_milliseconds;

uint32_t g_my_before, g_my_after, g_my_time_taken;
float32_t g_my_seconds, g_my_milliseconds;

const float SINGLE_CYCLE = 0.0000000625; //62.5 x 10^9
const int SEC_TO_MSEC = 10;

float times_faster = 0;

int main ()
{
   fpu_enable();

	/* Initilaize the uart*/
	uart2_tx_init();

//	serial_plot_impulse_response();
//
//	serial_plotter();

	/* Inilialize systick counter */
    systick_counter_init();

//	convolution(inputSignal_f32_1kHz_15kHz,destination_arr2,impulse_response,sig2_len,sig3_len );
//	serialplot_outputSig_convolved(destination_arr2);

    g_before = SysTick->VAL;

	 arm_conv_f32( inputSignal_f32_1kHz_15kHz,sig2_len,impulse_response,sig3_len,destination_arr3);
	 //serialplot_outputSig_convolved(destination_arr3);


   g_after = SysTick->VAL;

   /*Compute time taken*/
   g_time_taken = g_before - g_after;

   g_seconds = SINGLE_CYCLE * g_time_taken;

   g_milliseconds = g_seconds *SEC_TO_MSEC ;

   //my algo
   systick_counter_init();
   g_my_before = SysTick->VAL;

   convolution(inputSignal_f32_1kHz_15kHz,destination_arr2,impulse_response,sig2_len,sig3_len );

   g_my_after = SysTick->VAL;

   g_my_time_taken = g_my_before - g_my_after;

     g_my_seconds = SINGLE_CYCLE * g_my_time_taken;

     g_my_milliseconds = g_my_seconds *SEC_TO_MSEC ;

     //how much times cimsis algo is better?
     times_faster = g_my_milliseconds / g_milliseconds;
	/*Using CMSIS Library*/
	while(1)
	{
		//printf("Hello from stm32...\n\r");
		//plot_input_signal();
		//serial_plotter();
		//plot_impulse_response();

	}
}
void serialplot_outputSig_convolved(float32_t* destination_array)
{
//	for(int i=0; i<sig2_len+sig3_len; i++)
//		{
//		    printf("%f\r\n",destination_array[i] );
//			//printf("%ld\r\n", (long)(_5hz_signal[i] * 100000));
//			sudo_delay(9000);
//		}
	for(int i=0; i<sig2_len+sig3_len; i++)
	    {
	        long val = (long)(destination_array[i] * 100000);
	        long int_part  = val / 100000;
	        long frac_part = labs(val % 100000);

	        if (val < 0 && int_part == 0) {
	            printf("-%ld.%05ld\r\n", int_part, frac_part);
	        } else {
	            printf("%ld.%05ld\r\n", int_part, frac_part);
	        }

	        sudo_delay(9000);
	    }
}
//void convolution(float32_t* sig_arr, float32_t* destination_array,float32_t* imp_resp,
//		uint32_t sig_src_len, uint32_t imp_resp_len)
//{
//	uint32_t i,j;
//	/*compute output signal length*/
//	uint32_t final_sig_len = sig_src_len + imp_resp_len -1;
//
//	//clear output signal buffer
//	for(i=0;i< final_sig_len; i++)
//	{
//		destination_array[i] = 0;
//	}
//	//performing convolution
//	//Take every input sample x[i], multiply it by every impulse-response sample h[j],
//	//and put that contribution at output location i+j.
//
//	for(i = 0; i< final_sig_len; i++)
//	{
//		for(j = 0; j< imp_resp_len;j++)
//		{
//			destination_array[i+j] = destination_array[i+j] + sig_arr[i]*imp_resp[j];
//		}
//	}
//}
void convolution(float32_t *sig_arr,
                 float32_t *destination_array,
                 float32_t *imp_resp,
                 uint32_t sig_src_len,
                 uint32_t imp_resp_len)
{
    uint32_t i, j;

    uint32_t final_sig_len =
        sig_src_len + imp_resp_len - 1;

    for (i = 0; i < final_sig_len; i++)
    {
        destination_array[i] = 0.0f;
    }

    for (i = 0; i < sig_src_len; i++)
    {
        for (j = 0; j < imp_resp_len; j++)
        {
            destination_array[i + j] +=
                sig_arr[i] * imp_resp[j];
        }
    }
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

static void serial_plotter(void)
{
	for(int i=0; i<sig2_len; i++)
	{
	    printf("%f\r\n",inputSignal_f32_1kHz_15kHz[i] );
		//printf("%ld\r\n", (long)(_5hz_signal[i] * 100000));
		sudo_delay(9000);
	}
}

static void plot_input_signal(void)
{
	int i;
	for(i=0; i<sig2_len; i++)
	{
		in_sig_sample = inputSignal_f32_1kHz_15kHz[i];
		sudo_delay(9000);
	}
}
static void sudo_delay(uint16_t dly)
{
	for(volatile int j = 0; j<dly; j++){}
}

static void plot_impulse_response(void)
{
	for(int i = 0; i< sig3_len; i++)
	{
		imp_rsp_sample = impulse_response[i];
		sudo_delay(9000);
	}
}
static void serial_plot_impulse_response(void)
{
	for(int i=0; i<sig3_len; i++)
		{
		    printf("%f\r\n",impulse_response[i] );
			//printf("%ld\r\n", (long)(_5hz_signal[i] * 100000));
			sudo_delay(9000);
		}

}
