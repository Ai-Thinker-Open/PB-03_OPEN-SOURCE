
#ifndef __INCLUDE_CIRCBUF_H
#define __INCLUDE_CIRCBUF_H

/*  Note about locking: There is no locking required while only one reader
    and one writer is using the circular buffer.
    For multiple writer and one reader there is only a need to lock the
    writer. And vice versa for only one writer and multiple reader there is
    only a need to lock the reader.
*/

/****************************************************************************
    Included Files
 ****************************************************************************/

#include <stdbool.h>
#include "types.h"

/****************************************************************************
    Public Types
 ****************************************************************************/

/* This structure describes circular buffer */

struct circbuf_s
{
    void* base;     /* The pointer to buffer space */
    uint32_t    size;     /* The size of buffer space */
    uint32_t    head;     /* The head of buffer space */
    uint32_t    tail;     /* The tail of buffer space */
    bool      external; /* The flag for external buffer */
};

/****************************************************************************
    Public Function Prototypes
 ****************************************************************************/


/****************************************************************************
    Name: circbuf_init

    Description:
     Initialize a circular buffer.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
     base  - A pointer to circular buffer's internal buffer. It can be
             provided by caller because sometimes the creation of buffer
             is special or needs to preallocated, eg: DMA buffer.
             If NULL, a buffer of the given size will be allocated.
     bytes - The size of the internal buffer.

    Returned Value:
     Zero on success; A negated errno value is returned on any failure.

 ****************************************************************************/

int circbuf_init(struct circbuf_s* circ,
                 void* base, uint32_t bytes);


/****************************************************************************
    Name: circbuf_uninit

    Description:
     Free the circular buffer.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

void circbuf_uninit(struct circbuf_s* circ);

/****************************************************************************
    Name: circbuf_reset

    Description:
     Remove the entire circular buffer content.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

void circbuf_reset(struct circbuf_s* circ);

/****************************************************************************
    Name: circbuf_is_full

    Description:
     Return true if the circular buffer is full.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

bool circbuf_is_full(struct circbuf_s* circ);

/****************************************************************************
    Name: circbuf_is_empty

    Description:
     Return true if the circular buffer is empty.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

bool circbuf_is_empty(struct circbuf_s* circ);

/****************************************************************************
    Name: circbuf_size

    Description:
     Return size of the circular buffer.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

uint32_t circbuf_size(struct circbuf_s* circ);

/****************************************************************************
    Name: circbuf_used

    Description:
     Return the used bytes of the circular buffer.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

uint32_t circbuf_used(struct circbuf_s* circ);

/****************************************************************************
    Name: circbuf_space

    Description:
     Return the remaining space of the circular buffer.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

uint32_t circbuf_space(struct circbuf_s* circ);

/****************************************************************************
    Name: circbuf_peek

    Description:
     Get data form the circular buffer without removing

    Note :
     That with only one concurrent reader and one concurrent writer,
     you don't need extra locking to use these api.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
     dst   - Address where to store the data.
     bytes - Number of bytes to get.

    Returned Value:
     The bytes of get data is returned if the peek data is successful;
     A negated errno value is returned on any failure.
 ****************************************************************************/

ssize_t circbuf_peek(struct circbuf_s* circ,
                     void* dst, uint32_t bytes);

/****************************************************************************
    Name: circbuf_read

    Description:
     Get data form the circular buffer.

    Note :
     That with only one concurrent reader and one concurrent writer,
     you don't need extra locking to use these api.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
     dst   - Address where to store the data.
     bytes - Number of bytes to get.

    Returned Value:
     The bytes of get data is returned if the read data is successful;
     A negated errno value is returned on any failure.
 ****************************************************************************/

ssize_t circbuf_read(struct circbuf_s* circ,
                     void* dst, uint32_t bytes);

/****************************************************************************
    Name: circbuf_skip

    Description:
     Skip data form the circular buffer.

    Note:
     That with only one concurrent reader and one concurrent writer,
     you don't need extra locking to use these api.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
     bytes - Number of bytes to skip.

    Returned Value:
     The bytes of get data is returned if the skip data is successful;
     A negated errno value is returned on any failure.
 ****************************************************************************/

ssize_t circbuf_skip(struct circbuf_s* circ, uint32_t bytes);

/****************************************************************************
    Name: circbuf_write

    Description:
     Write data to the circular buffer.

    Note:
     That with only one concurrent reader and one concurrent writer,
     you don't need extra locking to use these api.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
     src   - The data to be added.
     bytes - Number of bytes to be added.

    Returned Value:
     The bytes of get data is returned if the write data is successful;
     A negated errno value is returned on any failure.
 ****************************************************************************/

ssize_t circbuf_write(struct circbuf_s* circ,
                      const void* src, uint32_t bytes);

/****************************************************************************
    Name: circbuf_overwrite

    Description:
     Write data to the circular buffer. It can overwrite old data when
     circular buffer don't have enough space to store data.

    Note:
     Usage circbuf_overwrite () is dangerous. It should be only called
     when the buffer is exclusived locked or when it is secured that no
     other thread is accessing the buffer.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
     src   - The data to be added.
     bytes - Number of bytes to be added.

    Returned Value:
     The bytes length of overwrite is returned if it's successful;
     A negated errno value is returned on any failure.
 ****************************************************************************/

ssize_t circbuf_overwrite(struct circbuf_s* circ,
                          const void* src, uint32_t bytes);

#endif

