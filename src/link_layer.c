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
static LinkLayer linkLayer;
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
unsigned char readControlFrameWithTimeout(void) {
    State state = START;
    unsigned char b, A = 0, C = 0;

    // Do NOT start or stop alarms here — the caller handles them.
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
                    C = b;
                    state = C_RCV;
                    break;
                case C_RCV:
                    if (b == (A ^ C)) state = BCC1_OK;
                    else if (b == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC1_OK:
                    if (b == FLAG) {
                        // Got a valid supervision frame
                        return C;
                    } else {
                        state = START;
                    }
                    break;
                default:
                    break;
            }
        }
    }

    // If we got here, timeout triggered (alarmFlag == TRUE)
    return 0;
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
    linkLayer = connectionParameters;
    printf("[DBG llopen] enter llopen, role=%d, serialPort='%s', baud=%d, timeout=%d, nRetrans=%d\n",
           connectionParameters.role,
           connectionParameters.serialPort,
           connectionParameters.baudRate,
           connectionParameters.timeout,
           connectionParameters.nRetransmissions);
    fflush(stdout);

    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
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
        // -------------------------------
        // TRANSMITTER
        // -------------------------------
        case LlTx: {

            int attempts = 0;

            while (attempts < nRetransmissions && state != STOP) {
                sendSupervisionFrame(fd, A_SE, C_SET);
                printf("[DBG llopen TX] Sent SET frame (attempt %d/%d)\n",
                       attempts + 1, nRetransmissions);
                fflush(stdout);

                alarmFlag = FALSE;
                (void) signal(SIGALRM, alarmHandler); 
                alarm(timeout);

                while (state != STOP && !alarmFlag) {
                    if (readByteSerialPort(fd, &byte) > 0) {
                        printf("[DBG llopen] read returned=%d byte=0x%02X\n", 1, byte);
                        fflush(stdout);

                        switch (state) {
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
                                if (byte == FLAG) {
                                    state = STOP;
                                    (void) signal(SIGALRM, alarmHandler); 
                                    alarm(0);
                                    alarmFlag = FALSE;
                                    printf("[DBG llopen TX] Received UA -> Connection established\n");
                                    return fd;
                                } else state = START;
                                break;
                            default:
                                break;
                        }
                    }
                }

                if (state != STOP) {
                    if (alarmFlag)
                        printf("[DBG llopen TX] Timeout waiting for UA, retrying...\n");
                    attempts++;
                }
            }

            printf("[DBG llopen TX] Connection setup failed after %d attempts.\n", attempts);
            return -1;
        }

        // -------------------------------
        // RECEIVER
        // -------------------------------
        case LlRx: {
            printf("[DBG llopen RX] waiting for SET on fd=%d\n", fd);
            fflush(stdout);

            while (state != STOP) {
                if (readByteSerialPort(fd, &byte) > 0) {
                    printf("[DBG llopen] read returned=%d byte=0x%02X\n", 1, byte);
                    fflush(stdout);

                    switch (state) {
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
                            if (byte == FLAG) {
                                state = STOP;
                                (void) signal(SIGALRM, alarmHandler); 
                                alarm(0);
                                printf("[DBG llopen RX] Received SET -> sending UA\n");
                                sendSupervisionFrame(fd, A_RE, C_UA);
                                return fd;
                            } else state = START;
                            break;
                        default:
                            break;
                    }
                }
            }

            break;
        }

        default:
            printf("[DBG llopen] exiting llopen with fd=%d (invalid role)\n", fd);
            fflush(stdout);
            return -1;
    }

    printf("[DBG llopen] exiting llopen with fd=%d (stop state=%d)\n", fd, state);
    fflush(stdout);
    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////

int llwrite(const unsigned char *buf, int bufSize)
{
    static int Ns = 0;  // sequence number 0 or 1

    // Compute BCC2
    unsigned char bcc2 = BCC2(buf, bufSize);
    unsigned char *payload = malloc(bufSize + 1);
    if (!payload) { perror("malloc"); return -1; }
    memcpy(payload, buf, bufSize);
    payload[bufSize] = bcc2;

    // Byte stuffing
    unsigned char stuffedPayload[(bufSize + 1) * 2];
    int stuffedSize = byteStuffing(payload, bufSize + 1, stuffedPayload);
    free(payload);

    // Build full frame: FLAG A C BCC1 DATA FLAG
    unsigned char *frame = malloc(stuffedSize + 5);
    if (!frame) { perror("malloc"); return -1; }

    int pos = 0;
    frame[pos++] = FLAG;
    frame[pos++] = A_SE;
    frame[pos++] = (Ns == 0) ? CI0 : CI1;
    frame[pos++] = frame[1] ^ frame[2]; // BCC1
    memcpy(frame + pos, stuffedPayload, stuffedSize);
    pos += stuffedSize;
    frame[pos++] = FLAG;
    int totalSize = pos;

    int attempts = 0;
    int acknowledged = 0;

    while (attempts < nRetransmissions && !acknowledged) {
        alarmFlag = FALSE;

        // Send frame and start timer
        writeBytesSerialPort(fd, frame, totalSize);
        printf("[TX] Sent I(%d), attempt %d/%d\n", Ns, attempts + 1, nRetransmissions);
        fflush(stdout);

        (void) signal(SIGALRM, alarmHandler);
        alarm(timeout);

        while (!alarmFlag && !acknowledged) {
            unsigned char response = readControlFrameWithTimeout();
            if (response == 0) continue; // timeout still ticking

            if (response == rr_control((Ns + 1) % 2)) {
                printf("[TX] Received RR(%d) → ACK for I(%d)\n", (Ns + 1) % 2, Ns);
                Ns = (Ns + 1) % 2;
                acknowledged = 1;
                break;
            }
            else if (response == rej_control(Ns)) {
                printf("[TX] Received REJ(%d) → retransmit I(%d)\n", Ns, Ns);
                break; // retry immediately
            }
            else {
                printf("[TX] Unexpected control frame 0x%02X\n", response);
            }
        }

        alarm(0); // stop timer

        if (!acknowledged) {
            attempts++;
            printf("[TX] Retry %d/%d\n", attempts, nRetransmissions);
        }
    }

    free(frame);

    if (acknowledged) {
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
int llclose() {
    State state = START;
    unsigned char byte;
    int attempts = nRetransmissions; 
    (void) signal(SIGALRM, alarmHandler);

    printf("[DBG llclose] Starting link layer close sequence. Role = %s\n",
           (linkLayer.role == LlTx) ? "Transmitter" : "Receiver");

    // -------------------------------
    // TRANSMITTER SIDE (LlTx)
    // -------------------------------
    if (linkLayer.role == LlTx) {
        printf("[DBG llclose TX] Sending DISC to initiate disconnection.\n");

        while (attempts > 0 && state != STOP) {
            sendSupervisionFrame(fd, A_SE, C_DISC);
            alarmFlag = FALSE;
            alarm(timeout);

            while (!alarmFlag && state != STOP) {
                if (readByteSerialPort(fd, &byte) > 0) {
                    switch (state) {
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
                            if (byte == FLAG) {
                                state = STOP;
                                alarm(0);
                                alarmFlag = FALSE;
                                printf("[DBG llclose TX] Received DISC from RX, sending UA.\n");
                                sendSupervisionFrame(fd, A_SE, C_UA);
                                closeSerialPort();
                                return 0;
                            }
                            else state = START;
                            break;
                        default: break;
                    }
                }
            }

            if (alarmFlag) {
                printf("[DBG llclose TX] Timeout waiting for DISC, retrying...\n");
                attempts--;
            }
        }

        printf("[DBG llclose TX] Connection close timed out.\n");
        alarm(0);
        closeSerialPort();
        return -1;
    }

    // -------------------------------
    // RECEIVER SIDE (LlRx)
    // -------------------------------
    else if (linkLayer.role == LlRx) {
        printf("[DBG llclose RX] Waiting for DISC from transmitter.\n");

        alarmFlag = FALSE;
        (void) signal(SIGALRM, alarmHandler);
        alarm(timeout);  // Set timeout for DISC reception

        while (state != STOP && !alarmFlag) {
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
                        if (byte == C_DISC) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_SE ^ C_DISC)) state = BCC1_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC1_OK:
                        if (byte == FLAG) {
                            state = STOP;
                            alarm(0);
                            printf("[DBG llclose RX] Received DISC from TX.\n");
                        } else state = START;
                        break;
                    default: break;
                }
            }
        }

        if (alarmFlag) {
            printf("[DBG llclose RX] Timeout waiting for DISC → closing anyway.\n");
            closeSerialPort();
            return -1;
        }

        // Send DISC reply to TX
        sendSupervisionFrame(fd, A_RE, C_DISC);
        printf("[DBG llclose RX] Sent DISC reply to TX.\n");

        // Wait for UA
        state = START;
        alarmFlag = FALSE;
        (void) signal(SIGALRM, alarmHandler); 
        alarm(timeout);

        while (state != STOP && !alarmFlag) {
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
                        if (byte == C_UA) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_SE ^ C_UA)) state = BCC1_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC1_OK:
                        if (byte == FLAG) {
                            state = STOP;
                            (void) signal(SIGALRM, alarmHandler); 
                            alarm(0);
                            printf("[DBG llclose RX] Received UA, closing serial port.\n");
                            closeSerialPort();
                            return 0;
                        } else state = START;
                        break;
                    default: break;
                }
            }
        }
        (void) signal(SIGALRM, alarmHandler); 

        alarm(0);
        printf("[DBG llclose RX] Timeout waiting for UA, closing anyway.\n");
        closeSerialPort();
        return -1;
    }

    // -------------------------------
    // Invalid role
    // -------------------------------
    else {
        fprintf(stderr, "[ERR llclose] Unknown role, cannot close.\n");
        closeSerialPort();
        return -1;
    }
}