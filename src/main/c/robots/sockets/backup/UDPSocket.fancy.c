#include <stdio.h>
#include "X80ProUdpSocket.h"

UdpSocket new_UdpSocket(char* ipAddress)
{
	return connectRobot(ipAddress, DEFAULT_ROBOT_PORT, DEFAULT_MIN_TIME_STEP);
}

UdpSocket new_UdpSocket(char* ipAddress, int port)
{
	return connectRobot(ipAddress, port, DEFAULT_MIN_TIME_STEP);
}

UdpSocket new_UdpSocket(char* ipAddress, int port, int minTimeInMillis)
{
	return connectRobot(ipAddress, port, minTimeInMillis);
}

char* getRobotIp(UdpSocket this)
{
	return this->robotIp;
}

int getRobotPort(UdpSocket this)
{
	return this->robotPort;
}

void setRobotIp(UdpSocket this, char* robotIp)
{
	this->robotIp = robotIp;
}

void setRobotPort(UdpSocket this, int robotPort)
{
	this->robotPort = robotPort;
}

UdpSocket connectRobot(char* robotIp, int robotPort, int minTimeSlice)
{
	UdpSocket this = (UdpSocket)malloc(sizeof(UdpSocket));
	this->robotIp = robotIp;
	this->robotPort = robotPort;

	fprintf(stderr,"robotIp = %s", robotIp);
	fprintf(stderr,"robotPort = %d", robotPort);
	fprintf(stderr,"minTimeSlice = %d", minTimeSlice);

	struct addrinfo hints, *p;

	memset(&hints, 0, sizeof(struct addrinfo));
	hints.ai_family   = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
	hints.ai_socktype = SOCK_DGRAM;   /* Datagram socket */
	hints.ai_flags    = 0;
	hints.ai_protocol = 0;            /* Any protocol */

	int s;
	if ((s = getaddrinfo(robotIp, robotPort, &hints, &this->addrInfo)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
		exit(1);
	}

	for(p = this->addrInfo; p != null; p = this->addrInfo->ai_next) {
		if((this->socket = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
			 	 perror("talker: socket");
		}
	}

	return this;
}

int send(UdpSocket this, Buffer buf)
{
	int result;
	if ((result = sendto(this->sock, buf->data, buf->size(buf), 0, this->ai_addr, this->ai_addr->ai_addrlen)) == -1) {
	        perror("talker: sendto");
	        exit(1);
	    }
	delete_Buffer(cmd);
	return result;
}
