#ifndef _RING_BUFF_H_
#define _RING_BUFF_H_

#define buff_size 100

#include <stdbool.h>

typedef struct RingBuf{
    char *rd_ptr;
    char *wrt_ptr;
    bool full;
    char array[buff_size];
}RingBuf;

void RingBuf_init(RingBuf* rb);

char RingBuf_read(RingBuf* rb);

void RingBuf_write(RingBuf* rb, char newChar);


#endif
