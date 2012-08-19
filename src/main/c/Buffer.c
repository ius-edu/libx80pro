#include "Buffer.h"

Buffer* new_Buffer(int size)
{
    Buffer* buf = (Buffer*)malloc(sizeof(Buffer));
    buf->data = (byte*)calloc(size, sizeof(byte));

    buf->size = size;

    return buf;
}

void delete_Buffer(Buffer* buf)
{
    free(buf->data);
    free(buf);
}

static int size(Buffer* buf)
{
    return buf->length;
}
