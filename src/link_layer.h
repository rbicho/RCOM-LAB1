// Link layer header.
// DO NOT CHANGE THIS FILE

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_

typedef enum
{
    LlTx,
    LlRx,
} LinkLayerRole;

typedef struct
{
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;

// Size of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer.
#define MAX_PAYLOAD_SIZE 1000

// MISC
#define FALSE 0
#define TRUE 1

// Open a connection using the "port" parameters defined in struct linkLayer.
// Return 0 on success or -1 on error.
int llopen(LinkLayer connectionParameters);

// Send data in buf with size bufSize.
// Return number of chars written, or -1 on error.
int llwrite(const unsigned char *buf, int bufSize);

// Receive data in packet.
// Return number of chars read, or -1 on error.
int llread(unsigned char *packet);

// Close previously opened connection and print transmission statistics in the console.
// Return 0 on success or -1 on error.
int llclose();

// Handles timeout alarms triggered by SIGALRM.
void alarmHandler(int signal);

// Reads a supervision/control frame (e.g., SET, UA, DISC, RR, REJ) from the serial port.
// Returns the control byte (C) if a valid frame is received, or 0 if it times out or is invalid.
unsigned char readControlFrameWithTimeout(void);

// Calculates the BCC2 used for data integrity verification.
unsigned char BCC2(const unsigned char *data, int size);

// Builds and sends a supervision/control frame through the serial port.
// Returns the number of bytes written to the serial port.
int sendSupervisionFrame(int fd, unsigned char addressField, unsigned char controlField);

// Applies byte stuffing to the data field to ensure frame transparency.
// Returns the number of bytes written to the output buffer.
int byteStuffing(const unsigned char *input, int inputSize, unsigned char *output);

// Reverses the byte stuffing applied during transmission.
// Returns the number of bytes written to the output buffer.
int byteDestuffing(const unsigned char *input, int inputSize, unsigned char *output);

// Displays transmission statistics 
void showStatistics()


#endif // _LINK_LAYER_H_
