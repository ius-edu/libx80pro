#include <stdio.h>
#include "UDPSocket.h"

UDPSocket sock;

static void UDPSocket_connectRobot(char *ipaddr, int udpport)
{
	socket.ipaddr = ipaddr;
	socket.udpport = udpport;

	fprintf(stderr, "ipaddr = %s", ipaddr);
	fprintf(stderr, "udpport = %d", udpport);
	/* fprintf(stderr, "delay = %d", delay); */

	struct addrinfo hints;
	struct addrinfo *p;

	memset(&hints, 0, sizeof(struct addrinfo));

	hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
	hints.ai_socktype = SOCK_DGRAM;   /* Datagram socket */
	/* hints.ai_flags = 0; */
	hints.ai_protocol = 0;            /* Any protocol */

	int s;
	if (0 != (s = getaddrinfo(ipaddr, udpport, &hints, &sock.addrInfo)))
	{
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
		exit(1);
	}

	for (p = sock.addrInfo; p != null; p = sock.addrInfo->ai_next) 
	{
		if (-1 == (sock.socket = socket(p->ai_family, p->ai_socktype, p->ai_protocol)))
		{
			perror("socket error");
		}
	}

	return socket;
}

static int UDPSocket_send(Datagram* datagram)
{
	int result;

	if (-1 == (result = sendto(sock->sock, datagram.data, datagram.size, 0, sock->ai_addr, sock->ai_addr->ai_addrlen)))
	{
		perror("talker: sendto");
		exit(1);
	}

	return result;
}
