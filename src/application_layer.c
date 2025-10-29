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
    linkLayer.role = (strcmp(role, "tx") == 0) ? LlTx : LlRx;

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
        if(fseek(file, 0, SEEK_END) != 0){
            perror("applicationLayer: fseek");
            fclose(file);
            llclose();
            return;
        }
        long fileLen = ftell(file);
        rewind(file);
        unsigned int ctrlSize = 0;
        unsigned char* startPacket = createControlPacket(C_START, filename, fileLen, &ctrlSize);
        if(llwrite(startPacket, ctrlSize) == -1){
            fprintf(stderr, "applicationLayer: llwrite START falhou\n");
            free(startPacket);
            fclose(file);
            llclose();
            return;
        }
        free(startPacket);

        unsigned char buffer[MAX_PAYLOAD_SIZE];
        unsigned char seq = 0;
        long totalSent = 0;
        int packetsSent = 0;

        size_t r;
        while ((r = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
            int pktSize = 0;
            unsigned char *dataPkt = createDataPacket(seq, buffer, (int)r, &pktSize);
            if (!dataPkt) {
                fprintf(stderr, "applicationLayer: createDataPacket falhou\n");
                break;
            }

            if (llwrite(dataPkt, pktSize) < 0) {
                fprintf(stderr, "applicationLayer: llwrite (data) falhou\n");
                free(dataPkt);
                break;
            }

            free(dataPkt);
            seq = (unsigned char)((seq + 1) & 0xFF);
            totalSent += (long)r;
            packetsSent++;
        }

        if (ferror(file)) {
            fprintf(stderr, "applicationLayer: erro durante leitura do ficheiro\n");
            fclose(file);
            llclose();
            return;
        }

        fclose(file);

        unsigned char *endPkt = createControlPacket(C_END, filename, (unsigned long)fileLen, &ctrlSize);
        if (!endPkt) {
            fprintf(stderr, "applicationLayer: createControlPacket END falhou\n");
            llclose();
            return;
        }
        if (llwrite(endPkt, (int)ctrlSize) < 0) {
            fprintf(stderr, "applicationLayer: llwrite END falhou\n");
            free(endPkt);
            llclose();
            return;
        }
        free(endPkt);

        printf("TX: enviado %ld bytes em %d pacotes\n", totalSent, packetsSent);

    }
    else {
        unsigned char rxPacket[MAX_PAYLOAD_SIZE];
        int rxLen;
        FILE *out = NULL;
        unsigned long expectedSize = 0;
        char *remoteName = NULL;
        long receivedBytes = 0;

        while (1) {
            rxLen = llread(rxPacket);
            if (rxLen < 0) {
                fprintf(stderr, "applicationLayer: llread devolveu erro\n");
                break;
            }
            if (rxLen == 0) {
                continue;
            }

            unsigned char C = rxPacket[0];

            if (C == C_START) {
                unsigned long fileSizeFromPkt = 0;
                unsigned char *name = parseControlPacket(rxPacket, rxLen, &fileSizeFromPkt);
                if (!name) {
                    fprintf(stderr, "applicationLayer: parseControlPacket START falhou\n");
                    continue;
                }
                if (out) { fclose(out); out = NULL; }
                out = fopen((char*)name, "wb");
                if (!out) {
                    perror("applicationLayer: fopen (receber)");
                    free(name);
                    break;
                }
                expectedSize = fileSizeFromPkt;
                remoteName = malloc(strlen((char*)name) + 1);
                strcpy(remoteName, (char*)name);
                receivedBytes = 0;
                printf("RX: START filename='%s' size=%lu\n", name, fileSizeFromPkt);
                free(name);
            }
            else if (C == C_DATA) {
                if (!out) {
                    fprintf(stderr, "applicationLayer: recebido DATA mas START ainda não chegou -> ignora\n");
                    continue;
                }
                unsigned char payload[MAX_PAYLOAD_SIZE];
                int payloadLen = parseDataPacket(rxPacket, (unsigned int)rxLen, payload);
                if (payloadLen < 0) {
                    fprintf(stderr, "applicationLayer: parseDataPacket falhou\n");
                    continue;
                }
                size_t w = fwrite(payload, 1, (size_t)payloadLen, out);
                if (w != (size_t)payloadLen) {
                    perror("applicationLayer: fwrite falhou");
                    fclose(out);
                    out = NULL;
                    break;
                }
                receivedBytes += payloadLen;
            }
            else if (C == C_END) {
                unsigned long fileSizeFromPkt = 0;
                unsigned char *name = parseControlPacket(rxPacket, rxLen, &fileSizeFromPkt);
                if (name) {
                    printf("RX: END filename='%s' size=%lu\n", name, fileSizeFromPkt);
                    free(name);
                } else {
                    printf("RX: END (sem nome no TLV)\n");
                }
                if (out) {
                    fclose(out);
                    out = NULL;
                }
                printf("RX: recepção concluída, recebidos %ld bytes (esperados %lu)\n", receivedBytes, expectedSize);
                break;
            }
            else {
                fprintf(stderr, "applicationLayer: C desconhecido: %u\n", (unsigned int)C);
            }
        }
        if (out) fclose(out);
        if (remoteName) free(remoteName);
    }

    if (llclose() < 0) {
        fprintf(stderr, "applicationLayer: llclose devolveu erro\n");
    }
}

unsigned char* parseControlPacket(const unsigned char* packet, int size, unsigned long *fileSize)
{
    if (!packet || size <= 0 || !fileSize) return NULL;

    *fileSize = 0;
    unsigned char *filename = NULL;

    int i = 1;
    while (i + 1 < size) {
        unsigned char T = packet[i];
        unsigned char L = packet[i+1];
        if (i + 2 + L > size) {
            if (filename) { free(filename); }
            return NULL;
        }

        const unsigned char *V = packet + i + 2;

        if (T == T_FILESIZE) {
            unsigned long v = 0;
            int toRead = L;
            if (toRead > (int)sizeof(unsigned long)) toRead = (int)sizeof(unsigned long);
            for (int b = 0; b < toRead; ++b) {
                v = (v << 8) | (unsigned long)V[b];
            }
            *fileSize = v;
        }
        else if (T == T_FILENAME) {
            filename = (unsigned char*)malloc(L + 1);
            if (!filename) return NULL;
            memcpy(filename, V, L);
            filename[L] = '\0';
        }
        i += 2 + L;
    }

    return filename;
}


unsigned char* createControlPacket(unsigned char controlField, const char* filename, unsigned long length, unsigned int* packetSize)
{
    if (!filename || !packetSize) return NULL;
    unsigned long tmp = length;
    int L_len = 1;
    while (tmp > 0xFF && L_len < (int)sizeof(unsigned long)) {
        tmp >>= 8;
        L_len++;
    }

    int L_name = (int)strlen(filename);
    *packetSize = 1 + (1 + 1 + L_len) + (1 + 1 + L_name);

    unsigned char* packet = (unsigned char*)malloc(*packetSize);
    if (!packet) return NULL;

    int pos = 0;
    packet[pos++] = (unsigned char)controlField;

    packet[pos++] = (unsigned char)T_FILESIZE;
    packet[pos++] = (unsigned char)L_len;
    for (int b = 0; b < L_len; ++b) {
        int shift = 8 * (L_len - 1 - b);
        packet[pos++] = (unsigned char)((length >> shift) & 0xFF);
    }

    packet[pos++] = (unsigned char)T_FILENAME;
    packet[pos++] = (unsigned char)L_name;
    memcpy(packet + pos, filename, L_name);
    pos += L_name;

    return packet;
}

unsigned char* createDataPacket(unsigned char sequence, const unsigned char *data, int dataSize, int *packetSize)
{
    if (!data || dataSize < 0 || dataSize > 65535 || !packetSize) return NULL;

    *packetSize = 1 + 1 + 2 + dataSize;
    unsigned char *packet = (unsigned char*)malloc((size_t)*packetSize);
    if (!packet) return NULL;

    packet[0] = (unsigned char)C_DATA;
    packet[1] = sequence;
    packet[2] = (unsigned char)((dataSize >> 8) & 0xFF);
    packet[3] = (unsigned char)(dataSize & 0xFF);
    if (dataSize > 0) memcpy(packet + 4, data, (size_t)dataSize);

    return packet;
}

unsigned char* getData(FILE* fd, long int fileLength)
{
    if (!fd) return NULL;
    if (fileLength <= 0) return NULL;

    unsigned char *buf = (unsigned char*)malloc((size_t)fileLength);
    if (!buf) return NULL;

    size_t read = fread(buf, 1, (size_t)fileLength, fd);
    if (read != (size_t)fileLength) {
        free(buf);
        return NULL;
    }
    return buf;
}


int parseDataPacket(const unsigned char* packet, const unsigned int packetSize, unsigned char* buffer)
{
    if (!packet || packetSize < 4 || !buffer) return -1;
    if (packet[0] != (unsigned char)C_DATA) return -1;

    unsigned int length = ((unsigned int)packet[2] << 8) | (unsigned int)packet[3];
    if (packetSize < 4 + length) return -1;

    if (length > 0) memcpy(buffer, packet + 4, length);
    return (int)length;
}