#pragma once
#ifndef UDPSOCKET_H
#define UDPSOCKET_H
#include <sys/types.h>
#include <sys/socket.h>

typedef struct UdpSocket
{
	struct addrinfo *addrinfo;
    char*  robotIp;
    int    robotPort;
    int    socket;
    Buffer rxBuf;
    Buffer rxPkt;
    Buffer txBuf;
    Buffer txPkt;
}   *UdpSocket;

#endif
