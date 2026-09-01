#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>
#include <stdbool.h>

#define RXFIFO_Size     550
#define RXFIFO_Fail     0
#define RXFIFO_Done     1

typedef uint16_t rx_DataType;

/* Extern declarations so main.c can use them */
extern rx_DataType RX_FIFO[RXFIFO_Size];
extern volatile rx_DataType *rx_put_pt;
extern volatile rx_DataType *rx_get_pt;

void rx_fifo_init(void);
uint8_t rx_fifo_put(rx_DataType data);
uint8_t rx_fifo_get(rx_DataType *datapt);
bool rx_fifo_is_empty(void);
bool rx_fifo_is_full(void);

#endif
