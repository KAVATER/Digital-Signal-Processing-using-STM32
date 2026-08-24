#include "Ring_Buff.h"

#include <stdio.h>
#include <stdlib.h>
#include "string.h"
#include <stdbool.h>

void RingBuf_init(RingBuf* rb)
{
    rb->rd_ptr = &rb->array[0];
    rb->wrt_ptr = &rb->array[0];
    rb->full = false;
}

void RingBuf_write(RingBuf* rb, char newChar)
{
    if(rb->full == true)
    {
        rb->rd_ptr+=1;
    }
    if(rb->rd_ptr > &(rb->array[buff_size-1]))
    {
        rb->rd_ptr = &(rb->array[0]);
    }

    //writing buffer
    *(rb->wrt_ptr) = newChar;
     rb->wrt_ptr += 1;

    if(rb->wrt_ptr > &(rb->array[buff_size-1]))
    {
      rb->wrt_ptr = &(rb->array[0]);
    }
    if(rb-> wrt_ptr == rb-> rd_ptr)
    {
        rb->full = true;
    }
}

char RingBuf_read(RingBuf* rb)
{
    char ch = '\0';
if (rb->rd_ptr != rb->wrt_ptr || rb->full)
{
  ch = *(rb->rd_ptr);
  rb->rd_ptr += 1;
  rb->full = false;

  if(rb->rd_ptr > &(rb->array[buff_size-1]))
  {
    rb->rd_ptr = &(rb->array[0]);
  }
}
//return ch;
}
