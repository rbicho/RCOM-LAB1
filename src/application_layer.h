// Application layer protocol header.
// DO NOT CHANGE THIS FILE

#ifndef _APPLICATION_LAYER_H_
#define _APPLICATION_LAYER_H_

#include "stdio.h"

#define T_FILESIZE 0
#define T_FILENAME 1

#define C_START 1
#define C_DATA  2
#define C_END   3

// Application layer main function.
// Arguments:
//   serialPort: Serial port name (e.g., /dev/ttyS0).
//   role: Application role {"tx", "rx"}.
//   baudrate: Baudrate of the serial port.
//   nTries: Maximum number of frame retries.
//   timeout: Frame timeout.
//   filename: Name of the file to send / receive.
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename);

// Parses a data packet and extracts the filename and file size.
// Arguments:
// packet: Pointer to the data packet.
// packetSize: Size of the data packet.
// buffer: Pointer to store the extracted data payload.
int parseDataPacket(const unsigned char* packet, const unsigned int packetSize, unsigned char* buffer);

// Parses a control packet and extracts the filename and file size.
// Arguments: 
// packet: Pointer to the control packet data.
// size: Size of the control packet.
// fileSize: Pointer to store the extracted file size.
unsigned char* parseControlPacket(const unsigned char* packet, int size, unsigned long *fileSize);

// Creates a data packet with the given sequence number and data.
// Arguments:
// sequence: Sequence number of the data packet.
// data: Pointer to the data to be included in the packet.
// dataSize: Size of the data.
unsigned char* createDataPacket(unsigned char sequence, const unsigned char* data, int dataSize, int* packetSize);

// Creates a control packet with the given control field, filename, and file size.
// Arguments:
// controlField: Control field value (C_START or C_END).
// filename: Name of the file.
// fileSize: Size of the file.
// packetSize: Pointer to store the size of the created packet.
unsigned char* createControlPacket(unsigned char controlField, const char* filename, unsigned long fileSize, unsigned int* packetSize);

// Parses a data packet and extracts the payload into the provided buffer.
// Arguments:
// packet: Pointer to the data packet.
// packetSize: Size of the data packet.
int parseDataPacket(const unsigned char* packet, const unsigned int packetSize, unsigned char* buffer);

// Reads data from a file into a buffer.
// Arguments:
// file: Pointer to the opened file.
// fileLength: Length of data to read from the file.
unsigned char* getData(FILE* file, long fileLength);

#endif // _APPLICATION_LAYER_H_
