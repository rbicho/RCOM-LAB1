// Link layer protocol implementation

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#include "main.c"
#include "link_layer.h"
#include "application_layer.c"
#include "serial_port.h"
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>


#define FLAG 0x7E
#define A_SE 0x03
#define A_RE 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define BCC(a,b) ((a) ^ (b))
#define CI0 0x00
#define CI1 0x80

int alarmFlag = FALSE;
int alarmCount = 0;
int timeout = 0;
int nRetransmissions = 0;
int Ns = 0; // Send sequence number


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


#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55

// Escaping
#define ESC 0x7D
#define ESC_7E 0x5E
#define ESC_7D 0x5D

// Helper to build control bytes
unsigned char rr_control(int next_expected_ns) {
    return next_expected_ns ? C_RR1 : C_RR0;
}
unsigned char rej_control(int expected_ns) {
    return expected_ns ? C_REJ1 : C_REJ0;
}

//HELPER FUNCTIONS

int sendSupervisionFrame(unsigned char A, unsigned char C){

    unsigned char frame[5];
    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = C;
    frame[3] = BCC(frame[1], frame[2]);
    frame[4] = FLAG;

    return writeBytesSerialPort(frame,5);
}

// returns control byte C if a valid supervision frame is received (with proper A and BCC1), or 0 on timeout/error
unsigned char readControlFrameWithTimeout(int timeout_seconds) {
    State state = START;
    unsigned char b;
    (void) signal(SIGALRM, alarmHandler);
    alarm(timeout_seconds);
    alarmFlag = FALSE;
    unsigned char A, C;
    while (!alarmFlag) {
        if (readByteSerialPort(&b) > 0) {
            switch (state) {
                case START:
                    if (b == FLAG) st = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (b == A_SE || b == A_RE) { A = b; st = A_RCV; }
                    else if (b == FLAG) st = FLAG_RCV;
                    else st = START;
                    break;
                case A_RCV:
                    C = b; st = C_RCV;
                    break;
                case C_RCV:
                    if (b == (A ^ C)) st = BCC1_OK;
                    else if (b == FLAG) { st = FLAG_RCV; }
                    else st = START;
                    break;
                case BCC1_OK:
                    if (b == FLAG) {
                        alarm(0);
                        return C;
                    } else {
                        st = START;
                    }
                    break;
                default: break;
            }
        }
    }
    // timeout
    return 0;
}


int BCC2(const unsigned char *data, int size){
    unsigned char bcc2 = data[0];
    for (int i = 1; i < size; i++) {
        bcc2 ^= data[i];
    }
    return bcc2;
}

void alarmHandler(int signal){
    alarmFlag = TRUE;
    alarmCount++;
}

int byteStuffing(const unsigned char *input, int inputSize, unsigned char *output){
    int j = 0;
    for (int i = 0; i < inputSize; i++) {
        if (input[i] == FLAG) {
            output[j++] = ESC;
            output[j++] = ESC_7E;
        } else if (input[i] == ESC) {
            output[j++] = ESC;  
            output[j++] = ESC_7D;
        } else {
            output[j++] = input[i];
        }
    }
    return j; // Return the size of the stuffed output
}

int byteDestuffing(const unsigned char *input, int inputSize, unsigned char *output){
    int j = 0;
    for (int i = 0; i < inputSize; i++) {
        if (input[i] == ESC) {
            i++;
            if (i < inputSize) {
                if (input[i] == ESC_7E) {
                    output[j++] = FLAG;
                } else if (input[i] == ESC_7D) {
                    output[j++] = ESC;
                } else {
                    // Invalid escape sequence, handle error as needed
                }
            }
        } else {
            output[j++] = input[i];
        }
    }
    return j; // Return the size of the destuffed output
}



////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    int fd = openSerialPort(connectionParameters.serialPort,connectionParameters.baudRate);
    if (fd<0) return -1;
    State state = START;

    unsigned char byte;
    int timeout = connectionParameters.timeout;
    int nRetransmissions = connectionParameters.nRetransmissions;

    switch (connectionParameters.role)
    {
        case LlTx:{

            (void) signal(SIGALRM, alarmHandler);

            while(connectionParameters.nRetransmissions > 0 && state != STOP){

                sendSupervisionFrame(A_SE, C_SET);
                alarm(timeout);
                alarmFlag = FALSE;

                while(state != STOP && !alarmFlag){
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
                                    alarmFlag = FALSE;
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
                if (readByteSerialPort(&byte) > 0){
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
                                return fd;
                            }
                            else state = START;
                            break;
                        default:
                            break;
                    }  
                }
            }
            sendSupervisionFrame(A_RE, C_UA); 
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

    if (!frame) {
        perror("Memory allocation failed");
        return -1;
    }

    //Ns = (Ns + 1) % 2; maybe use this logic later

    frame[0] = FLAG;
    frame[1] = A_SE;
    frame[2] = (Ns == 0) ? CI0 : CI1;   // use current Ns value
    frame[3] = frame[1] ^ frame[2];

    memcpy(frame+4, buf, bufSize);
    unsigned char BCC2 = buf[0];


    // Usar funcao BCC2
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



    int currentransmitions = 0;
    int rejected=0;
    int accepted=0;

    while (nRetransmissions >= currentransmitions){
        alarmFlag=FALSE;
        alarm(timeout);
        rejected=0;
        accepted=0;

        while (!alarmFlag && !rejected && !accepted){
            writeBytesSerialPort(frame, framesize);
            unsigned char response = readControlFrameWithTimeout(timeout);

            if (response == C_RR1 || response == C_RR0){
                accepted=1;
            }
            else if (response == C_REJ0 || response == C_REJ1){
                rejected=1;

            }
            else continue;

        }
        if (accepted){
            break;
            currentransmitions++;
        }
        
    }

    free (frame);
    if (accepted) return framesize;
    else{
        llclose();
        return -1;
    }
}

    








    

    



////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    unsigned char byte;
    unsigned char controlField;
    int i =0;

    State state = START;

    while (state != STOP){
        if (readByteSerialPort(&byte) > 0){
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
                        if (byte == C_SET || byte == C_UA){
                        state = C_RCV;
                        controlField = byte;   
                    }
                    else if (byte == FLAG) state = FLAG_RCV;
                    else if (byte == C_DISC) {
                        sendSupervisionFrame(A_RE, C_DISC);
                        return 0;
                    }
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == BCC(A_SE, controlField)) state = BCC1_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case DATA_READING:
                    if (byte == ESC) state = DATA_ESC;
                    else if (byte == FLAG){
                        unsigned char bcc2 = packet[i-1];
                        i--;
                        packet[i] = '\0';
                        unsigned char acc = packet[0];

                        for (unsigned int j = 1; j < i; j++)
                            acc ^= packet[j];

                        if (bcc2 == acc){
                            state = STOP;
                            sendSupervisionFrame(A_RE, C_RR(NS));
                            return i; 
                        }
                        else{
                            printf("Error: retransmition\n");
                            sendSupervisionFrame(A_RE, C_REJ(NS));
                            return -1;
                        };

                    }
                    else{
                        packet[i++] = byte;
                    }
                    break;
                case DATA_ESC
                    state = DATA_READING;
                    if (byte == ESC || byte == FLAG) packet[i++] = byte;
                    else{
                        packet[i++] = ESC;
                        packet[i++] = byte;
                    }
                    break;
                default: 
                    break;
                
                
            }
        }  
    }
    return 0;
}

 



////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(){
    State state = START;
    unsigned char byte;
    (void) signal(SIGALRM, alarmHandler);

    while (nRetransmissions != 0 && state != STOP){

        sendSupervisionFrame(A_SE, C_DISC);
        alarm(timeout);
        alarmFlag = FALSE;

        while(state != STOP && !alarmFlag){
            if (readByteSerialPort(&byte) > 0){
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
                        if (byte == C_DISC) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_RE ^ C_DISC)) state = BCC1_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC1_OK:
                        if (byte == FLAG){
                            state = STOP;
                            alarm(0);
                            alarmFlag = FALSE;
                            sendSupervisionFrame(A_SE, C_UA);
                            closeSerialPort();
                            return 0;
                        }
                        else state = START;
                        break;
                    default:
                        break;
                }  
            }
        }
        nRetransmissions--;
    }
    if (state != STOP) return -1;
    sendSupervisionFrame(A_ER, C_UA);
    return closeSerialPort();

}
