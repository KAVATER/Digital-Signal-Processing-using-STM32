#ifndef _FIFO_H_
#define _FIFO_H_
#include "stdint.h"

#define RXFIFO_Size 300
#define RXFIFO_Fail 0
#define RXFIFO_Done 1

typedef uint32_t rx_DataType;
volatile void rx_fifo_init(void);
uint8_t rx_fifo_put(rx_DataType data);
uint8_t rx_fifo_get(rx_DataType * datapt);




#endif
