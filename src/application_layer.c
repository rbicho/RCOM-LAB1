// Application layer protocol implementation

#include "application_layer.h"

#include <stdio.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer linkLayer;
    strcpy(linkLayer.serialPort, serialPort);
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;
    //linkLayer.role = (role[0]== "LlTx") ? LlTx : LlRx

    llopen(linkLayer);

    if (linkLayer.role == LlTx)
    {
        //Transmitter
        llwrite(NULL);
    }
    else
    {
        //Receiver
        llread(NULL);
    }

}
