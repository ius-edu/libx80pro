#pragma once
#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#include <sys/types.h>
#include <sys/socket.h>

typedef unsigned char BYTE;

static DEFAULT_PORT = 10001;
static DEFAULT_DELAY = 70; /* milliseconds */
static CONNECT_WAIT = 5000; /* milliseconds */

typedef struct
{
	int DEFAULT_PORT;
	int DEFAULT_DELAY;
	int CONNECT_WAIT;
	
	struct addrinfo *addrInfo;
    char ip[16];
    int port;
    int socket;
    BYTE rxBuf[1024];
    BYTE rxPkt[2048];
    BYTE txBuf[256];
    BYTE txPkt[512];
	
	/* methods */
	int (*connect)(struct UDPSocket *self, (*sensorEvent)(unsigned char *sensorData), char *ipAddress, int port);
	int (*send)(struct UDPSocket *self, Buffer *buf);
	void (*recv)(struct UDPSocket *self, Buffer *buf);
} UDPSocket;

extern UDPSocket_init(UDPSocket *self);

#endif
