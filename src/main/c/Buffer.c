#include <stdlib.h>
#include "Buffer.h"

static int size(Buffer* buf)
{
    return buf->length;
}

void delete_Buffer(Buffer* buf)
{
    free(buf->data);
    free(buf);
}

Buffer* new_Buffer(int len)
{
    Buffer* buf = (Buffer*)malloc(sizeof(Buffer));
    buf->data = (unsigned char*)calloc(len, sizeof(unsigned char));

    buf->size = size;

    return buf;
}
