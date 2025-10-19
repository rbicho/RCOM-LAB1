// Application layer protocol implementation

#include "application_layer.h"
#include <stdio.h>
#include "link_layer.h"
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer linkLayer;
    strcpy(linkLayer.serialPort, serialPort);
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;
    linkLayer.role = strcmp(role, "LlTx") ? LlTx : LlRx;

    if (llopen(linkLayer) < 0)
    {
        printf("Error\n");
        return;
    }

    if (linkLayer.role == LlTx)
    {
        FILE *file = fopen(filename, "rb");
        if (!file) {
                perror("Failed opening file\n");
                exit(-1);
            }
        //Transmitter
        llwrite(NULL);
    }
    else
    {
        //Receiver
        llread(NULL);
    }
}