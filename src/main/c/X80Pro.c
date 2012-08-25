#include "boards/PMS5005.h"
#include "sockets/X80ProUdpSocket.h"
#include "X80Pro.h"
#include "Buffer.h"

void delete_X80Pro(X80Pro* self)
{
    free(self);
}

X80Pro new_X80Pro()
{
    X80Pro self = (X80Pro)malloc(sizeof(X80Pro));
    
    self->pms5005 = new_PMS5005();
    self->socket = new_X80ProUdpSocket();

    return self;
}
