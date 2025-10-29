// Link layer protocol implementation

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#include "link_layer.h"
#include "application_layer.h"
#include "serial_port.h"
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>



#define FLAG 0x7E
#define A_SE 0x03 // Sender to Receiver
#define A_RE 0x01 //Receiver to Sender
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define BCC(a,b) ((a) ^ (b))
#define CI0 0x00
#define CI1 0x80

static int fd = -1;
int alarmFlag = FALSE;
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
    //DATA_ESC,
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

int sendSupervisionFrame(int fd, unsigned char A, unsigned char C){

    unsigned char frame[5];
    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = C;
    frame[3] = BCC(frame[1], frame[2]);
    frame[4] = FLAG;

    int res = writeBytesSerialPort(fd, frame, 5);
    printf("[DBG sendSupervisionFrame] Wrote %d bytes: %02X %02X %02X %02X %02X\n",
           res, frame[0], frame[1], frame[2], frame[3], frame[4]);
    fflush(stdout);

    return res;
}

// returns control byte C if a valid supervision frame is received (with proper A and BCC1), or 0 on timeout/error
unsigned char readControlFrameWithTimeout(int timeout_seconds) {
    State state = START;
    unsigned char b;
    (void) signal(SIGALRM, alarmHandler);
    alarmFlag = FALSE;
    alarm(timeout_seconds);
    unsigned char A = 0, C = 0;

    while (!alarmFlag) {
        if (readByteSerialPort(fd, &b) > 0) {
            switch (state) {
                case START:
                    if (b == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (b == A_SE || b == A_RE) { A = b; state = A_RCV; }
                    else if (b == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case A_RCV:
                    C = b; state = C_RCV;
                    break;
                case C_RCV:
                    if (b == (A ^ C)) state = BCC1_OK;
                    else if (b == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC1_OK:
                    if (b == FLAG) {
                        alarm(0);
                        return C;
                    } else {
                        state = START;
                    }
                    break;
                default: break;
            }
        }
    }
    return 0; // timeout
}


unsigned char BCC2(const unsigned char *data, int size){
    if (size <= 0) return 0;
    unsigned char bcc2 = data[0];
    for (int i = 1; i < size; i++) {
        bcc2 ^= data[i];
    }
    return bcc2;
}

void alarmHandler(int signal){
    alarmFlag = TRUE;
    alarmCount++;
    printf("[DBG alarmHandler] alarm fired\n"); fflush(stdout);
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
    printf("[DBG llopen] enter llopen, role=%d, serialPort='%s', baud=%d, timeout=%d, nRetrans=%d\n",
       connectionParameters.role,
       connectionParameters.serialPort,
       connectionParameters.baudRate,
       connectionParameters.timeout,
       connectionParameters.nRetransmissions);
    fflush(stdout);
     fd = openSerialPort(connectionParameters.serialPort,connectionParameters.baudRate);
    if (fd < 0) {
    printf("[DBG llopen] openSerialPort FAILED, fd=%d\n", fd);
    fflush(stdout);
    return -1;
    }
    printf("[DBG llopen] serial opened fd=%d\n", fd);
    fflush(stdout);
    State state = START;

    unsigned char byte;
     timeout = connectionParameters.timeout;
     nRetransmissions = connectionParameters.nRetransmissions;

    switch (connectionParameters.role)
    {
        case LlTx:{

            (void) signal(SIGALRM, alarmHandler);

            while(nRetransmissions > 0 && state != STOP){

                sendSupervisionFrame(fd, A_SE, C_SET);
                printf("[DBG llopen TX] Sent SET frame on fd=%d\n", fd);
fflush(stdout);
                alarmFlag = FALSE;
                alarm(timeout);


                while(state != STOP && !alarmFlag){
                    if (readByteSerialPort(fd, &byte) > 0){
                        printf("[DBG llopen] read returned=%d byte=0x%02X\n", 1, byte);
 fflush(stdout);

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
                nRetransmissions--;
            }
        }
        if (state != STOP){
            return -1;
        }
        break;
        case LlRx:{
            printf("[DBG llopen RX] waiting for SET on fd=%d\n", fd);
fflush(stdout);

            while(state != STOP){
                if (readByteSerialPort(fd, &byte) > 0){
                    printf("[DBG llopen] read returned=%d byte=0x%02X\n", 1, byte);
 fflush(stdout);

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
                                alarm(0);
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
            
            break;
    }
    default:
    printf("[DBG llopen] exiting llopen with fd=%d (stop state=%d)\n", fd, state);
fflush(stdout);

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
    static int Ns = 0;  

    //  BCC2 over payload ---
    unsigned char bcc2 = BCC2(buf, bufSize);

    unsigned char *payload = (unsigned char *)malloc(bufSize + 1);
    if (!payload) {
        perror("malloc");
        return -1;
    }
    memcpy(payload, buf, bufSize);
    payload[bufSize] = bcc2;

    //byte stuffing
    unsigned char stuffedPayload[(bufSize + 1) * 2]; 
    int stuffedSize = byteStuffing(payload, bufSize + 1, stuffedPayload);
    free(payload);

    // frame header
    unsigned char *frame = (unsigned char *)malloc(stuffedSize + 5); // FLAG A C BCC1 ... FLAG
    if (!frame) {
        perror("malloc");
        return -1;
    }

    int pos = 0;
    frame[pos++] = FLAG;
    frame[pos++] = A_SE;
    frame[pos++] = (Ns == 0) ? CI0 : CI1;
    frame[pos++] = frame[1] ^ frame[2];  // BCC1

    // Insert stuffed data 
    memcpy(frame + pos, stuffedPayload, stuffedSize);
    pos += stuffedSize;

    // end glag
    frame[pos++] = FLAG;
    int totalSize = pos;

    int attempts = 0;
    int accepted = 0;

    while (attempts < nRetransmissions && !accepted) {
        alarmFlag = FALSE;
        alarm(timeout);

        // Send frame
        writeBytesSerialPort(fd, frame, totalSize);
        printf("[TX] Sent I(%d), attempt %d\n", Ns, attempts + 1);

        while (!alarmFlag && !accepted) {
            unsigned char response = readControlFrameWithTimeout(timeout);
            if (response == 0)
                continue; 

            // ACK
            if (response == rr_control((Ns + 1) % 2)) {
                printf("[TX] Received RR(%d) → ACK for I(%d)\n", (Ns + 1) % 2, Ns);
                Ns = (Ns + 1) % 2;  
                accepted = 1;
                break;
            }
            // NACK
            else if (response == rej_control(Ns)) {
                printf("[TX] Received REJ(%d) → retransmit I(%d)\n", Ns, Ns);
                break; 
            } 
            else {
                printf("[TX] Ignoring unexpected response 0x%02X\n", response);
            }
        }

        if (!accepted) {
            attempts++;
            printf("[TX] Retry %d/%d\n", attempts, nRetransmissions);
        }
    }

    free(frame);

    if (accepted) {
        alarm(0);
        printf("[TX] Frame acknowledged successfully.\n");
        return totalSize;
    } else {
        printf("[TX] Transmission failed after %d attempts.\n", attempts);
        llclose();
        return -1;
    }
}







    

    



////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    static int expectedNs = 0;

    unsigned char byte;
    unsigned char controlField = 0;
    int i = 0;
    State state = START;

    while (state != STOP) {
        if (readByteSerialPort(fd, &byte) > 0) {
            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;

                case FLAG_RCV:
                    if (byte == A_SE) state = A_RCV;
                    else if (byte != FLAG) state = START;
                    break;

                case A_RCV:
                    // I frame control field (CI0 or CI1)
                    if (byte == CI0 || byte == CI1) {
                        controlField = byte;
                        state = C_RCV;
                    } else if (byte == C_DISC) {
                        sendSupervisionFrame(fd, A_RE, C_DISC);
                        return 0;
                    } else if (byte == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;

                case C_RCV:
                    if (byte == (A_SE ^ controlField))
                        state = DATA_READING;
                    else if (byte == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;

                case DATA_READING:
                    if (byte == FLAG) {
                        // Dynamically allocate enough space for destuffing
                        unsigned char *destuffed = (unsigned char *)malloc(i);
                        if (!destuffed) {
                            perror("malloc failed");
                            sendSupervisionFrame(fd, A_RE, rej_control(expectedNs));
                            return -1;
                        }

                        int destuffedLen = byteDestuffing(packet, i, destuffed);
                        if (destuffedLen < 1) {
                            free(destuffed);
                            sendSupervisionFrame(fd, A_RE, rej_control(expectedNs));
                            return -1;
                        }

                        // Verify BCC2
                        unsigned char receivedBCC2 = destuffed[destuffedLen - 1];
                        unsigned char computedBCC2 = BCC2(destuffed, destuffedLen - 1);
                        unsigned int receivedNs = (controlField == CI1) ? 1 : 0;

                        if (receivedBCC2 == computedBCC2) {
                            if (receivedNs == expectedNs) {
                                sendSupervisionFrame(fd, A_RE, rr_control((expectedNs + 1) % 2));
                                expectedNs = (expectedNs + 1) % 2;
                                memcpy(packet, destuffed, destuffedLen - 1);
                                free(destuffed);
                                return destuffedLen - 1;
                            } else {
                                sendSupervisionFrame(fd, A_RE, rr_control(expectedNs));
                                free(destuffed);
                                return 0;
                            }
                        } else {
                            printf("[RX] BCC2 error → REJ(%d)\n", expectedNs);
                            sendSupervisionFrame(fd, A_RE, rej_control(expectedNs));
                            free(destuffed);
                            return -1;
                        }
                    } else {
                        packet[i++] = byte;
                    }
                    break;

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
int llclose(){
    State state = START;
    unsigned char byte;
    (void) signal(SIGALRM, alarmHandler);

    while (nRetransmissions != 0 && state != STOP){

        sendSupervisionFrame(fd, A_SE, C_DISC);
        alarmFlag = FALSE;
        alarm(timeout);

        while(state != STOP && !alarmFlag){
            if (readByteSerialPort(fd, &byte) > 0){
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
                            sendSupervisionFrame(fd, A_SE, C_UA);
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
    sendSupervisionFrame(fd, A_SE, C_UA);
    return closeSerialPort();

}
