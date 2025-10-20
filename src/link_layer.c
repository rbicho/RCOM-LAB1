// Link layer protocol implementation

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#include "main.c"
#include "link_layer.h"
#include "application_layer.c"
#include "serial_port.c"
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

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
#define BCC(a,b) ((a) ^ (b))
#define CI0 0x00
#define CI1 0x80

int alarmFLag = FALSE;
int alarmCount = 0;
int timeout = 0;
int nRetransmissions = 0;


typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC1_OK,
    STOP,
    DATA_ESC,
    DATA_READING,
    DISC,
    BCC2_OK,

} State;

State state = START;

//HELPER FUNCTIONS
int sendSupervisionFrame(int fd, unsigned char A, unsigned char C){
    unsigned char frame[5];
    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = C;
    frame[3] = BCC(frame[1], frame[2]);
    frame[4] = FLAG;

    return writeBytesSerialPort(frame,5);
}

unsigned char readControlFrame(int fd);

int BCC2(const unsigned char *data, int size);

void alarmHandler(int signal);




////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    int fd = openSerialPort(connectionParameters.serialPort,connectionParameters.baudRate);
    if (fd<0) return -1;
    unsigned char byte;
    int timeout = connectionParameters.timeout;
    int nRetransmissions = connectionParameters.nRetransmissions;

    switch (connectionParameters.role)
    {
        case LlTx:{

            while(connectionParameters.nRetransmissions > 0 && state != STOP){

                sendSupervisionFrame(fd, A_SE, C_SET);
                alarm(timeout);
                state = START;

                while(state != STOP && !alarmFLag){
                    if (readByteSerialPort(&byte,1) > 0){
                        switch (state)
                        {
                            case START:
                                if (byte == FLAG) state = FLAG_RCV;
                                break;
                            case FLAG_RCV:
                                if (byte == A_RE) state = A_RCV;
                                else if (byte != FLAG) state = START;
                                break;
                            case A_RCV:
                                if (byte == C_UA) state = C_RCV;
                                else if (byte == FLAG) state = FLAG_RCV;
                                else state = START;
                                break;
                            case C_RCV:
                                if (byte == (A_RE ^ C_UA)) state = BCC1_OK;
                                else if (byte == FLAG) state = FLAG_RCV;
                                else state = START;
                                break;
                            case BCC1_OK:
                                if (byte == FLAG){
                                    state = STOP;
                                    alarm(0);
                                    alarmFLag = FALSE;
                                    return fd;
                                }
                                else state = START;
                                break;
                            default:
                                break;
                        }  
                    }
                }
                connectionParameters.nRetransmissions--;
            }
        }
        if (state != STOP){
            return -1;
        }
        break;
        case LlRx:{
            while(state != STOP){
                if (readByteSerialPort(&byte,1) > 0){
                    switch (state)
                    {
                        case START:
                            if (byte == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte == A_SE) state = A_RCV;
                            else if (byte != FLAG) state = START;
                            break;
                        case A_RCV:
                            if (byte == C_SET) state = C_RCV;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case C_RCV:
                            if (byte == (A_SE ^ C_SET)) state = BCC1_OK;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC1_OK:
                            if (byte == FLAG){
                                state = STOP;
                                sendSupervisionFrame(fd, A_RE, C_UA);
                                return fd;
                            }
                            else state = START;
                            break;
                        default:
                            break;
                    }  
                }
            }
            sendSupervisionFrame(fd, A_RE, C_UA);
            break;

    }
    default:
        return -1;
        break;

 }
 return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    int framesize= bufSize + 6; // FLAG A C BCC1 BCC2 FLAG
    unsigned char *frame = (unsigned char*)malloc(framesize);

    frame[0] = FLAG;
    frame[1] = A_SE;
    frame[2] = CI0; 
    frame[3] = frame[1] ^ frame[2]; 

    memcpy(frame+4, buf, bufSize);
    unsigned char BCC2 = buf[0];

    for (int i = 1; i < bufSize; i++) {
        BCC2 ^= buf[i];
    }

    int j = 4;

    for (int i=0;i<bufSize;i++){
        if (buf[i] == FLAG || buf[i] == DATA_ESC){
            frame = realloc(frame, ++framesize);
            frame[j++] = DATA_ESC;
        }
        else{
            frame[j++] = buf[i];
        }
    }
    frame[j++] = BCC2;
    frame[j++] = FLAG;

    // Transmitions logic -- review later

    int currentransmitions = 0;
    int rejected=0;
    int accepted=0;

    while (nRetransmissions >= currentransmitions){
        writeBytesSerialPort(frame, framesize);
        alarm(timeout);
        state = START;
        rejected=0;
        accepted=0;

        while (!alarmFLag && !rejected && !accepted){
            writeBytesSerialPort(frame, framesize);
            
            
           
        }
        if (accepted){
            alarmFLag=FALSE;
            break;
        }
        currentransmitions++;
        }
        free (frame);
        if (accepted) return framesize;
        else{
            llclose();
            return -1;
        }
        




    return bufSize;
}

    








    

    



////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    unsigned char byte;
    unsigned char controlField;

    State state = START;

    while (state != STOP){
        if (readByteSerialPort(&byte,1) > 0){
            switch (state)
            {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A_SE) state = A_RCV;
                    else if (byte != FLAG) state = START;
                    break;
                case A_RCV:
                    controlField = byte;
                    state = C_RCV;
                    break;
                case C_RCV:
                    if (byte == (A_SE ^ controlField)) state = BCC1_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC1_OK: {
                    // Read data until FLAG is encountered
                    int index = 0;
                    while (TRUE) {
                        if (readByteSerialPort(&byte, 1) > 0) {
                            if (byte == FLAG) {
                                state = STOP;
                                return index; // Return the size of the packet
                            } else {
                                packet[index++] = byte; // Store data byte
                            }
                        }
                    }
                    break;
                }
                default:
                    break;
            }  
        }
    }
    return -1;

}


////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    State state = START;
    unsigned char byte;
    (void) signal(SIGALRM, alarmHandler);
    

    return 0;
}
