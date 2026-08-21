#include "fifo.h"

rx_DataType RX_FIFO[RXFIFO_Size];

volatile rx_DataType * rx_put_pt;
volatile rx_DataType * rx_get_pt;

// Initializing fifo
 void rx_fifo_init(void)
{
	rx_put_pt = rx_get_pt = &RX_FIFO[0];
}
rx_DataType volatile *rx_next_put_pt;
//putting data into rx fifo
uint8_t rx_fifo_put(rx_DataType data)
{


	rx_next_put_pt = rx_put_pt +1;

	/*Check if at end*/
	if(rx_next_put_pt  == &RX_FIFO[RXFIFO_Size])
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
		*(rx_put_pt) = data;
		rx_put_pt = rx_next_put_pt;
		return (RXFIFO_Done);

	}
}

//get data from fifo
uint8_t rx_fifo_get(rx_DataType * datapt)
{
	if(rx_put_pt == rx_get_pt)
	{
		//fifo empty
		return (RXFIFO_Fail);
	}

	//get the data
	*datapt = *(rx_get_pt++);
	rx_get_pt++;

	//check if at the end
	if(rx_get_pt == &RX_FIFO[RXFIFO_Size])
	{
		//wrap around
		rx_get_pt = &RX_FIFO[0];
	}
	return (RXFIFO_Done);
}
