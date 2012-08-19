#pragma once
#ifndef BUFFER_H
#define BUFFER_H

typedef struct Buffer
{
    byte* data;
    int length;

    int *(size)(Buffer*);
} Buffer;

Buffer* new_Buffer(int);
void delete_Buffer(Buffer*);

#endif
