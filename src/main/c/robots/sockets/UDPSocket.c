#include <stdio.h>
#include <pthread.h>
#include "UDPSocket.h"

UDPSocket sock;

static int connectRobot(UDPSocket *self, char *ip, int port)
{
	int result = 0;
	
	self->ip = ip;
	self->port = port;

	fprintf(stderr, "robotIP: %s", ip);
	fprintf(stderr, "robotPort: %d", port);
	/* fprintf(stderr, "delay = %d", delay); */

	struct addrinfo hints;
	struct addrinfo *p;

	memset(&hints, 0, sizeof(struct addrInfo));

	hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
	hints.ai_socktype = SOCK_DGRAM;   /* Datagram socket */
	/* hints.ai_flags = 0; */
	hints.ai_protocol = 0;            /* Any protocol */

	int s;
	if (0 != (s = getaddrinfo(ip, port, &hints, &sock.addrInfo)))
	{
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
		result = -1;
		exit(1);
	}

	for (p = sock.addrInfo; p != null; p = sock.addrInfo->ai_next) 
	{
		if (-1 == (sock.socket = socket(p->ai_family, p->ai_socktype, p->ai_protocol)))
		{
			perror("socket error");
			result = -1;
		}
	}
	
	pthread_create();
	return socket;
}

static int send(Buffer *buf)
{
	int result;

	if (-1 == (result = sendto(sock->sock, buf.data, buf.length, 0, sock->ai_addr, sock->ai_addr->ai_addrlen)))
	{
		perror("talker: sendto");
		exit(1);
	}

	return result;
}

void *recv()
{
	
}

void close(UDPSocket *self)
{
	self->isFinished = true;
	//self->socket.close();
}

extern UDPSocket_init(UDPSocket *self)
{
	self->DEFAULT_ROBOT_PORT = DEFAULT_ROBOT_PORT;
	self->DEFAULT_TIME_STEP_IN_MS = DEFAULT_TIME_STEP_IN_MS;
	self->CONNECT_WAIT = CONNECT_WAIT;
	
	/* methods */
	self->connectRobot = connectRobot;
	self->send = send;
}
