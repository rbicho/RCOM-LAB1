// Application layer protocol implementation

#include "application_layer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "link_layer.h"
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer linkLayer;
    strcpy(linkLayer.serialPort, serialPort);
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;
    linkLayer.role = (strcmp(role, "LlTx") == 0) ? LlTx : LlRx;

    if (llopen(linkLayer) < 0)
    {
        printf("Failed to open link layer.\n");
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
    llclose(linkLayer);

}