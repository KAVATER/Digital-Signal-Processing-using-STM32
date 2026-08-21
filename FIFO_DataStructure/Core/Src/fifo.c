#include "fifo.h"

rx_DataType RX_FIFO[RXFIFO_Size];

volatile rx_DataType * rx_put_pt;
volatile rx_DataType * rx_get_pt;

// Initializing fifo
volatile rx_fifo_init(void)
{
	rx_put_pt = rx_get_pt = &RX_FIFO[0];
}

//putting data into rx fifo
uint8_t rx_fifo_put(rx_DataType data)
{
	rx_DataType volatile *rx_next_put_pt;

	rx_next_put_pt = rx_put_pt +1;

	/*Check if at end*/
	if(rx_next_put_pt  == &RXFIFO_Size[RXFIFOSIZE])
	{
		//wrap around
		rx_next_put_pt = &RX_FIFO[0];
	}
	if(rx_next_put_pt ==rx_get_pt)
	{
		return(RXFIFO_Fail);
	}
	else
	{
		//put data into fifo

	}
}
