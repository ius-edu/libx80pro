#pragma once

#ifndef BUFFER_H
#define BUFFER_H

typedef struct
{
    /* unsigned char data[65527]; */
	unsigned char data[32];
    int length;
} Buffer;

#endif
