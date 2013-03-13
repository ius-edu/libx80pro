#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include "UDPSocket.h"

static int connect(UDPSocket *self, char *ip, int port)
{
	int result = 0;
	
	self->ip = ip;
	self->port = port;
	
	fprintf(stderr, "ip: %s", ip);
	fprintf(stderr, "port: %d", port);
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
	
	/* pthread_create(); */
	return socket;
}

static int send(UDPSocket *self, Buffer *buf)
{
	int result;
	struct timespec tsi, tsf;
#if defined(_POSIX_TIMERS) && 0 < _POSIX_TIMERS
    clock_gettime(CLOCK_REALTIME, &ts);
#else
    struct timeval tvi;
    gettimeofday(&tvi, NULL);
    tsi.tv_sec = tvi.tv_sec;
    tsi.tv_nsec = 1000*tvi.tv_usec;
#endif
	if (-1 == (result = sendto(sock->sock, buf.data, buf.length, 0, sock->ai_addr, sock->ai_addr->ai_addrlen)))
	{
		perror("talker: sendto");
		exit(1);
	}
	nanosleep(tf.tv_nsec - (uint)ts.tv_nsec);
	return result;
}

void *recv(UDPSocket *self)
{
	
}

void close(UDPSocket *self)
{
	self->isFinished = true;
	//self->socket.close();
}

extern UDPSocket_init(UDPSocket *self)
{
	self->DEFAULT_PORT = DEFAULT_PORT;
	self->DEFAULT_DELAY = DEFAULT_DELAY;
	self->CONNECT_WAIT = CONNECT_WAIT;
	
	/* methods */
	self->connectRobot = connectRobot;
	self->send = send;
}
