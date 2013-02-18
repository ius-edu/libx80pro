#pragma once
#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#include <sys/types.h>
#include <sys/socket.h>

typedef unsigned char BYTE;

static DEFAULT_ROBOT_PORT = 10001;
static DEFAULT_TIME_STEP_IN_MS = 60; /* milliseconds */
static CONNECT_WAIT = 5; /* seconds */

typedef struct
{
	int DEFAULT_ROBOT_PORT;
	int DEFAULT_TIME_STEP_IN_MS;
	int CONNECT_WAIT;

	struct addrinfo *addrInfo;
    char robotIP[16];
    int robotPort;
    int socket;
    BYTE rxBuf[1024];
    BYTE rxPkt[2048];
    BYTE txBuf[256];
    BYTE txPkt[512];

	/* methods */
	int (*connectRobot)(struct UDPSocket *self, char *ipAddress, int port);
	int (*send)(struct UDPSocket *self, Buffer *buf);
} UDPSocket;

extern UDPSocket_init(UDPSocket *self);

#endif
