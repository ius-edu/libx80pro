#pragma once
#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#include <sys/types.h>
#include <sys/socket.h>

typedef unsigned char BYTE;

struct UDPSocket
{
	struct addrinfo *addrinfo;
    char ipaddr[16];
    int udpport;
    int socket;
    BYTE rxbuf[2048];
    BYTE rxpkt[2048];
    BYTE txbuf[2048];
    BYTE txpkt[2048];
};

#endif
