#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/stat.h>

#define BUFFER_SIZE 4096
#define RESPONSE_SIZE 1024
#define MAX_PATH 512

typedef struct {
    char user[50];
    char password[50];
    char host[100];
    char ip[16];
    char path[MAX_PATH];
    char filename[100];
    char directory[MAX_PATH];
} FTPInfo;

void parse_ftp_url(const char* url, FTPInfo* info) {
    strcpy(info->user, "anonymous");
    strcpy(info->password, "");
    
    if (strncmp(url, "ftp://", 6) != 0) {
        printf("ERROR: URL must start with 'ftp://'\n");
        exit(1);
    }
    
    const char* ptr = url + 6;
    
    const char* at_sign = NULL;
    const char* first_slash = strchr(ptr, '/');
    const char* search_end = first_slash ? first_slash : ptr + strlen(ptr);
    
    for (const char* p = ptr; p < search_end; p++) {
        if (*p == '@') {
            at_sign = p;
        }
    }
    
    if (at_sign != NULL) {
        const char* colon = strchr(ptr, ':');
        if (colon != NULL && colon < at_sign) {
            int user_len = colon - ptr;
            strncpy(info->user, ptr, user_len);
            info->user[user_len] = '\0';
            
            int pass_len = at_sign - (colon + 1);
            strncpy(info->password, colon + 1, pass_len);
            info->password[pass_len] = '\0';
        } else {
            int user_len = at_sign - ptr;
            strncpy(info->user, ptr, user_len);
            info->user[user_len] = '\0';
        }
        ptr = at_sign + 1;
    }
    
    if (first_slash == NULL) {
        strcpy(info->host, ptr);
        strcpy(info->path, "/");
        strcpy(info->filename, "");
        strcpy(info->directory, "/");
    } else {
        int host_len = first_slash - ptr;
        strncpy(info->host, ptr, host_len);
        info->host[host_len] = '\0';
        
        strcpy(info->path, first_slash);
        
        const char* last_slash = strrchr(info->path, '/');
        if (last_slash != NULL && *(last_slash + 1) != '\0') {
            strcpy(info->filename, last_slash + 1);
            
            int dir_len = last_slash - info->path;
            if (dir_len > 0) {
                strncpy(info->directory, info->path, dir_len);
                info->directory[dir_len] = '\0';
            } else {
                strcpy(info->directory, "/");
            }
        } else {
            strcpy(info->filename, "");
            strcpy(info->directory, info->path);
        }
    }
}

void get_ip(const char *hostname, char *ip) {
    struct hostent *h;

    if ((h = gethostbyname(hostname)) == NULL) {
        herror("gethostbyname");
        exit(1);
    }

    strcpy(ip, inet_ntoa(*((struct in_addr *) h->h_addr)));
}

int create_connection_socket(const char* ip, int port) {
    int sockfd;
    struct sockaddr_in server_addr;
    
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Error creating socket");
        return -1;
    }
    
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = inet_addr(ip);
    
    printf("Connecting to %s:%d...\n", ip, port);
    
    if (connect(sockfd, (struct sockaddr*) &server_addr, sizeof(server_addr)) < 0) {
        perror("Connection error");
        close(sockfd);
        return -1;
    }
    
    return sockfd;
}

int read_response(int sockfd, char* response, int response_size) {
    int total_bytes = 0;
    int bytes;
    char buffer[RESPONSE_SIZE];
    
    memset(response, 0, response_size);
    
    while (total_bytes < response_size - 1) {
        bytes = recv(sockfd, buffer, sizeof(buffer) - 1, 0);
        
        if (bytes <= 0) {
            if (total_bytes == 0) {
                return -1;
            }
            break;
        }
        
        buffer[bytes] = '\0';
        
        if (bytes >= 4 && buffer[3] == ' ') {
            strcat(response, buffer);
            total_bytes += bytes;
            break;
        }
        
        strcat(response, buffer);
        total_bytes += bytes;
    }
    
    printf("Server: %s", response);
    return total_bytes;
}

int send_command(int sockfd, const char* command) {
    printf("Client: %s", command);
    int bytes_sent = send(sockfd, command, strlen(command), 0);
    if (bytes_sent < 0) {
        perror("Error sending command");
        return -1;
    }
    return bytes_sent;
}

int get_response_code(const char* response) {
    if (strlen(response) >= 3) {
        return atoi(response);
    }
    return 0;
}

int ftp_login(int sockfd, const char* user, const char* password) {
    char response[RESPONSE_SIZE];
    
    char command[100];
    sprintf(command, "USER %s\r\n", user);
    if (send_command(sockfd, command) < 0) return -1;
    
    if (read_response(sockfd, response, sizeof(response)) < 0) return -1;
    int code = get_response_code(response);
    
    if (code == 331) {
        sprintf(command, "PASS %s\r\n", password);
        if (send_command(sockfd, command) < 0) return -1;
        
        if (read_response(sockfd, response, sizeof(response)) < 0) return -1;
        code = get_response_code(response);
    }
    
    if (code != 230) {
        printf("Login failed with code: %d\n", code);
        return -1;
    }
    
    printf("Login successful!\n");
    return 0;
}

int enter_passive_mode(int sockfd, char* data_ip, int* data_port) {
    char response[RESPONSE_SIZE];
    
    if (send_command(sockfd, "PASV\r\n") < 0) return -1;
    if (read_response(sockfd, response, sizeof(response)) < 0) return -1;
    
    char* start = strchr(response, '(');
    char* end = strchr(response, ')');
    
    if (!start || !end) {
        printf("Invalid PASV response format\n");
        return -1;
    }
    
    int nums[6];
    char* token = strtok(start + 1, ",");
    int i = 0;
    
    while (token != NULL && i < 6) {
        nums[i++] = atoi(token);
        token = strtok(NULL, ",");
    }
    
    if (i != 6) {
        printf("Failed to parse PASV data\n");
        return -1;
    }
    
    sprintf(data_ip, "%d.%d.%d.%d", nums[0], nums[1], nums[2], nums[3]);
    *data_port = nums[4] * 256 + nums[5];
    
    printf("Passive mode: IP=%s, Port=%d\n", data_ip, *data_port);
    return 0;
}

int download_with_fallback(int control_sock, int data_sock, FTPInfo* ftp_info) {
    char response[RESPONSE_SIZE];
    char command[512];
    int code;
    
    printf("\n=== Strategy 1: Full path RETR ===\n");
    sprintf(command, "RETR %s\r\n", ftp_info->path);
    
    if (send_command(control_sock, command) < 0) return -1;
    if (read_response(control_sock, response, sizeof(response)) < 0) return -1;
    
    code = get_response_code(response);
    
    if (code == 550) {
        printf("Strategy 1 failed (code %d). Trying Strategy 2...\n", code);
        
        if (strcmp(ftp_info->directory, "/") != 0 && strlen(ftp_info->directory) > 0) {
            printf("\n=== Strategy 2: CWD + filename RETR ===\n");
            
            sprintf(command, "CWD %s\r\n", ftp_info->directory);
            if (send_command(control_sock, command) < 0) return -1;
            if (read_response(control_sock, response, sizeof(response)) < 0) return -1;
            
            code = get_response_code(response);
            if (code == 250) {
                printf("Directory change successful\n");
                
                sprintf(command, "RETR %s\r\n", ftp_info->filename);
                if (send_command(control_sock, command) < 0) return -1;
                if (read_response(control_sock, response, sizeof(response)) < 0) return -1;
                
                code = get_response_code(response);
            }
        }
    }
    
    if (code != 150 && code != 125) {
        printf("All strategies failed. Last error code: %d\n", code);
        return -1;
    }
    
    printf("Starting file transfer...\n");
    
    FILE* file = fopen(ftp_info->filename, "wb");
    if (!file) {
        perror("Error creating local file");
        return -1;
    }
    
    char buffer[BUFFER_SIZE];
    int bytes;
    long total_bytes = 0;
    
    while ((bytes = recv(data_sock, buffer, sizeof(buffer), 0)) > 0) {
        fwrite(buffer, 1, bytes, file);
        total_bytes += bytes;
        printf("\rDownloaded: %ld bytes", total_bytes);
        fflush(stdout);
    }
    
    printf("\n");
    
    if (bytes < 0) {
        perror("Error receiving file data");
        fclose(file);
        return -1;
    }
    
    fclose(file);
    close(data_sock);
    
    if (read_response(control_sock, response, sizeof(response)) < 0) return -1;
    code = get_response_code(response);
    
    if (code != 226) {
        printf("Transfer incomplete. Code: %d\n", code);
        return -1;
    }
    
    struct stat file_stat;
    if (stat(ftp_info->filename, &file_stat) == 0) {
        printf("File '%s' downloaded successfully! Size: %ld bytes\n", 
               ftp_info->filename, file_stat.st_size);
    } else {
        printf("File downloaded but cannot verify size\n");
    }
    
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        printf("Usage: %s ftp://[user:password@]host/path\n", argv[0]);
        printf("\nExamples:\n");
        printf("  %s ftp://ftp.fe.up.pt/pub/README\n", argv[0]);
        printf("  %s ftp://anonymous:email@example.com@ftp.fe.up.pt/pub/networking/tcpdump/tcpdump.1.gz\n", argv[0]);
        printf("  %s ftp://rcom:rcom@netlab1.fe.up.pt/files/crab.mp4\n", argv[0]);
        return -1;
    }
    
    FTPInfo ftp_info;
    
    parse_ftp_url(argv[1], &ftp_info);
    
    printf("=== FTP Client ===\n");
    printf("User: %s\n", ftp_info.user);
    printf("Password: %s\n", (strlen(ftp_info.password) > 0) ? "***" : "(empty)");
    printf("Host: %s\n", ftp_info.host);
    printf("Full path: %s\n", ftp_info.path);
    printf("Directory: %s\n", ftp_info.directory);
    printf("Filename: %s\n", ftp_info.filename);
    printf("==================\n");
    
    char ip[16];
    get_ip(ftp_info.host, ip);
    strcpy(ftp_info.ip, ip);
    printf("Server IP: %s\n", ftp_info.ip);
    
    int control_sock = create_connection_socket(ftp_info.ip, 21);
    if (control_sock < 0) {
        printf("Failed to connect to FTP server\n");
        return -1;
    }
    
    char response[RESPONSE_SIZE];
    
    if (read_response(control_sock, response, sizeof(response)) < 0) {
        close(control_sock);
        return -1;
    }
    
    if (ftp_login(control_sock, ftp_info.user, ftp_info.password) < 0) {
        printf("Login failed\n");
        close(control_sock);
        return -1;
    }
    
    if (send_command(control_sock, "TYPE I\r\n") < 0) {
        close(control_sock);
        return -1;
    }
    read_response(control_sock, response, sizeof(response));
    
    char data_ip[16];
    int data_port;
    
    if (enter_passive_mode(control_sock, data_ip, &data_port) < 0) {
        printf("Failed to enter passive mode\n");
        close(control_sock);
        return -1;
    }
    
    int data_sock = create_connection_socket(data_ip, data_port);
    if (data_sock < 0) {
        printf("Failed to create data connection\n");
        close(control_sock);
        return -1;
    }
    
    printf("Data connection established\n");
    
    if (download_with_fallback(control_sock, data_sock, &ftp_info) < 0) {
        printf("File download failed\n");
        close(data_sock);
        close(control_sock);
        return -1;
    }
    
    send_command(control_sock, "QUIT\r\n");
    read_response(control_sock, response, sizeof(response));
    
    close(data_sock);
    close(control_sock);
    
    printf("Connection closed successfully.\n");
    return 0;
}