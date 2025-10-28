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

unsigned char* createControlPacket(unsigned char controlField, const char* filename, unsigned long fileSize, unsigned int* packetSize);

unsigned char* parseControlPacket(const unsigned char* packet, int size, unsigned long* fileSize);

unsigned char* createDataPacket(unsigned char sequence, const unsigned char* data, int dataSize, int* packetSize);

int parseDataPacket(const unsigned char* packet, const unsigned int packetSize, unsigned char* buffer);

unsigned char* getData(FILE* file, long fileLength);

#endif // _APPLICATION_LAYER_H_
