// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG 0x7E
#define A_SE 0x03
#define A_RE 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_DISC 0x0B
#define BCC(a,b) = a^b
#define CI0 0x00
#define CI1 0x80



////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    return openSerialPort(connectionParameters.serial_port, connectionParameters.baudRate);

}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    for (int i=0; i< bufSize; i++){
        writeBytesSerialPort(buf, bufSize);
    }
    
    return bufSize;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}
