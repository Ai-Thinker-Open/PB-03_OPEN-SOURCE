
/****************************************************************************
    Included Files
 ****************************************************************************/

#include "circular_buf.h"

/****************************************************************************
    Private Types
 ****************************************************************************/

/****************************************************************************
    Public Functions
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

int circbuf_init(struct circbuf_s* circ, void* base, uint32_t bytes)
{
    circ->external = !!base;
    circ->base = base;
    circ->size = bytes;
    circ->head = 0;
    circ->tail = 0;
    return 0;
}

/****************************************************************************
    Name: circbuf_reset

    Description:
     Remove the entire circular buffer content.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

void circbuf_reset(struct circbuf_s* circ)
{
    circ->head = circ->tail = 0;
}

/****************************************************************************
    Name: circbuf_uninit

    Description:
     Free the circular buffer.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

void circbuf_uninit(struct circbuf_s* circ)
{
    osal_memset(circ, 0, sizeof(*circ));
}

/****************************************************************************
    Name: circbuf_size

    Description:
     Return size of the circular buffer.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

uint32_t circbuf_size(struct circbuf_s* circ)
{
    return circ->size;
}

/****************************************************************************
    Name: circbuf_used

    Description:
     Return the used bytes of the circular buffer.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

uint32_t circbuf_used(struct circbuf_s* circ)
{
    return circ->head - circ->tail;
}

/****************************************************************************
    Name: circbuf_space

    Description:
     Return the remaining space of the circular buffer.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

uint32_t circbuf_space(struct circbuf_s* circ)
{
    return circbuf_size(circ) - circbuf_used(circ);
}

/****************************************************************************
    Name: circbuf_is_empty

    Description:
     Return true if the circular buffer is empty.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

bool circbuf_is_empty(struct circbuf_s* circ)
{
    return !circbuf_used(circ);
}

/****************************************************************************
    Name: circbuf_is_full

    Description:
     Return true if the circular buffer is full.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
 ****************************************************************************/

bool circbuf_is_full(struct circbuf_s* circ)
{
    return !circbuf_space(circ);
}

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

ssize_t circbuf_peek(struct circbuf_s* circ, void* dst, uint32_t bytes)
{
    uint32_t len;
    uint32_t off;

    if (!circ->size)
    {
        return 0;
    }

    len = circbuf_used(circ);
    off = circ->tail % circ->size;

    if (bytes > len)
    {
        bytes = len;
    }

    len = circ->size - off;

    if (bytes < len)
    {
        len = bytes;
    }

    osal_memcpy(dst, circ->base + off, len);
    osal_memcpy(dst + len, circ->base, bytes - len);
    return bytes;
}

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

ssize_t circbuf_read(struct circbuf_s* circ, void* dst, uint32_t bytes)
{
    bytes = circbuf_peek(circ, dst, bytes);
    circ->tail += bytes;
    return bytes;
}

/****************************************************************************
    Name: circbuf_skip

    Description:
     Skip data form the circular buffer.

    Note :
     That with only one concurrent reader and one concurrent writer,
     you don't need extra locking to use these api.

    Input Parameters:
     circ  - Address of the circular buffer to be used.
     bytes - Number of bytes to skip.

    Returned Value:
     The bytes of get data is returned if the skip data is successful;
     A negated errno value is returned on any failure.
 ****************************************************************************/

ssize_t circbuf_skip(struct circbuf_s* circ, uint32_t bytes)
{
    uint32_t len;
    len = circbuf_used(circ);

    if (bytes > len)
    {
        bytes = len;
    }

    circ->tail += bytes;
    return bytes;
}

/****************************************************************************
    Name: circbuf_write

    Description:
     Write data to the circular buffer.

    Note :
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

ssize_t circbuf_write(struct circbuf_s* circ, const void* src, uint32_t bytes)
{
    uint32_t space;
    uint32_t off;

    if (!circ->size)
    {
        return 0;
    }

    space = circbuf_space(circ);
    off = circ->head % circ->size;

    if (bytes > space)
    {
        bytes = space;
    }

    space = circ->size - off;

    if (bytes < space)
    {
        space = bytes;
    }

    osal_memcpy(circ->base + off, src, space);
    osal_memcpy(circ->base, src + space, bytes - space);
    circ->head += bytes;
    return bytes;
}

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

ssize_t circbuf_overwrite(struct circbuf_s* circ, const void* src, uint32_t bytes)
{
    uint32_t overwrite = 0;
    uint32_t space;
    uint32_t off;

    if (!circ->size)
    {
        return 0;
    }

    if (bytes > circ->size)
    {
        src += bytes - circ->size;
        bytes = circ->size;
    }

    space = circbuf_space(circ);

    if (bytes > space)
    {
        overwrite = bytes - space;
    }

    off = circ->head % circ->size;
    space = circ->size - off;

    if (bytes < space)
    {
        space = bytes;
    }

    osal_memcpy(circ->base + off, src, space);
    osal_memcpy(circ->base, src + space, bytes - space);
    circ->head += bytes;
    circ->tail += overwrite;
    return overwrite;
}

