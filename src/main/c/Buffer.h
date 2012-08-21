#pragma once
#ifndef BUFFER_H
#define BUFFER_H

typedef struct Buffer
{
    unsigned char* data;
    int length;

    int (*size)(struct Buffer*);
} Buffer;

Buffer* new_Buffer(int);
void delete_Buffer(Buffer*);

#endif
