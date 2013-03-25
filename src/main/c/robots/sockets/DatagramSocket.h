
typedef struct
{
	int soTimeOut;
	
	void (*setSoTimeOut)(int timeout);
	void (*send)(unsigned char *data);
	void (*receive)(DatagramSocket *);
	void (*setBuf)(unsigned char *buf, int length);
	void (*setDest)(char *server, int port);
} DatagramSocket;

extern DatagramSocket_init(DatagramSocket *self);
