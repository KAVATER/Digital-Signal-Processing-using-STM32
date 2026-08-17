#include "stm32f407xx.h"
#include "signals.h"
#include "uart.h"
#include "stdio.h"
#include "arm_math.h"
#include <math.h>

#define GPIOAEN  (1U<<0)
#define PIN5    (1U<<5)
#define LED_PIN  PIN5

//#define ITM_PORT0 (*(volatile uint32_t *)0xE0000000)

extern float _5hz_signal[sig1_len];
extern float32_t inputSignal_f32_1kHz_15kHz[sig2_len];

static void plot_input_signal(void);
static void sudo_delay(uint16_t dly);

float in_sig_sample;
static void fpu_enable(void);
static void serial_plotter(void);
static float32_t signal_mean(float32_t *sig_arr, uint32_t sig_len);
static float32_t signal_variance(float32_t* sig_arr, float32_t sig_mean, uint32_t sig_len);
static float32_t signal_standardDeviation(float32_t variance_local);


float32_t mean_val;
float32_t variance_val;
float32_t standard_deviation;

int main ()
{

   fpu_enable();

	/* Initilaize the uart*/
	uart2_tx_init();
	mean_val = signal_mean((float32_t*)inputSignal_f32_1kHz_15kHz,(uint32_t)sig2_len);
	variance_val = signal_variance((float32_t*)inputSignal_f32_1kHz_15kHz, (float32_t)mean_val, (uint32_t)sig2_len);
	standard_deviation = signal_standardDeviation(variance_val);

	while(1)
	{
		//printf("Hello from stm32...\n\r");
		//plot_input_signal();
		serial_plotter();
	}
}
static float32_t signal_standardDeviation(float32_t variance_local)
{
	return(sqrt(variance_local));
}
static float32_t signal_variance(float32_t* sig_arr, float32_t sig_mean, uint32_t sig_len)
{
	float32_t variance = 0.0;
	uint32_t i;
	for(i = 0; i<sig_len ; i++)
	{
		variance = variance + powf((sig_arr[i]- sig_mean),2);
	}
	variance = variance/(sig_len-1);
	return variance;
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
static float32_t signal_mean(float32_t *sig_arr, uint32_t sig_len)
{
	float32_t _mean = 0.0;
	uint32_t i;

	for (i=0; i<sig_len; i++)
	{
		_mean = _mean+sig_arr[i];
	}
	_mean = _mean/(float32_t)sig_len;
	return _mean;
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
