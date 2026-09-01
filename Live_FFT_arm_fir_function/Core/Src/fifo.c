#include "fifo.h"

rx_DataType RX_FIFO[RXFIFO_Size];

volatile rx_DataType *rx_put_pt;
volatile rx_DataType *rx_get_pt;
volatile uint32_t rx_fifo_count;

void rx_fifo_init(void)
{
    rx_put_pt = rx_get_pt = &RX_FIFO[0];
}

// fifo.c
uint8_t rx_fifo_put(rx_DataType data)
{
    if (rx_fifo_count >= RXFIFO_Size) return RXFIFO_Fail;   // truly full
    *rx_put_pt = data;
    rx_put_pt++;
    if (rx_put_pt == &RX_FIFO[RXFIFO_Size]) rx_put_pt = &RX_FIFO[0];
    rx_fifo_count++;
    return RXFIFO_Done;
}

uint8_t rx_fifo_get(rx_DataType *datapt)
{
    if (rx_fifo_count == 0) return RXFIFO_Fail;   // truly empty
    *datapt = *rx_get_pt;
    rx_get_pt++;
    if (rx_get_pt == &RX_FIFO[RXFIFO_Size]) rx_get_pt = &RX_FIFO[0];
    rx_fifo_count--;
    return RXFIFO_Done;
}

//uint8_t rx_fifo_put(rx_DataType data)
//{
//    volatile rx_DataType *rx_next_put_pt = rx_put_pt + 1;
//
//    /* Wrap around if at end */
//    if (rx_next_put_pt == &RX_FIFO[RXFIFO_Size])
//    {
//        rx_next_put_pt = &RX_FIFO[0];
//    }
//
//    /* Check if FIFO is full */
//    if (rx_next_put_pt == rx_get_pt)
//    {
//        return RXFIFO_Fail;
//    }
//
//    /* Put data and advance pointer */
//    *rx_put_pt = data;
//    rx_put_pt = rx_next_put_pt;
//    return RXFIFO_Done;
//}
//
//uint8_t rx_fifo_get(rx_DataType *datapt)
//{
//    /* Check if FIFO is empty */
//    if (rx_put_pt == rx_get_pt)
//    {
//        return RXFIFO_Fail;
//    }
//
//    /* Get data and advance pointer — ONLY ONE INCREMENT! */
//    *datapt = *rx_get_pt;
//    rx_get_pt++;
//
//    /* Wrap around if at end */
//    if (rx_get_pt == &RX_FIFO[RXFIFO_Size])
//    {
//        rx_get_pt = &RX_FIFO[0];
//    }
//
//    return RXFIFO_Done;
//}

bool rx_fifo_is_empty(void)
{
    return (rx_put_pt == rx_get_pt);
}

bool rx_fifo_is_full(void)
{
    volatile rx_DataType *next = rx_put_pt + 1;
    if (next == &RX_FIFO[RXFIFO_Size])
        next = &RX_FIFO[0];
    return (next == rx_get_pt);
}
