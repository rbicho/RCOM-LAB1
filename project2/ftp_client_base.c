#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

typedef struct {
    char user[50];
    char password[50];
    char host[100];
    char ip[16];
    char path[256];
    char filename[100];
} FTPInfo;

void parse_ftp_url(const char* url, FTPInfo* info){
    strcpy(info->user, "anonymous");
    strcpy(info->password, "");
    
    if(strncmp(url, "ftp://", 6) != 0){
        printf("ERROR: url must start with 'ftp://'\n");
        exit(1);
    }
    
    const char* ptr = url + 6;
    const char* at_sign = strchr(ptr, '@');
    
    if(at_sign != NULL){
        const char* colon = strchr(ptr, ':');
        if(colon != NULL && colon < at_sign){
            int user_len = colon - ptr;
            strncpy(info->user, ptr, user_len);
            info->user[user_len] = '\0';
            
            int pass_len = at_sign - (colon + 1);
            strncpy(info->password, colon + 1, pass_len);
            info->password[pass_len] = '\0';
        }
        ptr = at_sign +1;
    }
    
    const char* first_slash = strchr(ptr, '/');
    if (first_slash == NULL){
        strcpy(info->host, ptr);
        strcpy(info->path, "/");
        strcpy(info->filename, "");
    } 
    else{
        int host_len = first_slash - ptr;
        strncpy(info->host, ptr, host_len);
        info->host[host_len] = '\0';
        
        strcpy(info->path, first_slash);
        
        const char* last_slash = strrchr(info->path, '/');
        if (last_slash != NULL && *(last_slash + 1) != '\0') {
            strcpy(info->filename, last_slash + 1);
        } else {
            strcpy(info->filename, "");
        }
    }
}

char* get_ip(const char* hostname){
    struct hostent* h;
    if((h = gethostbyname(hostname)) == NULL){
        herror("DNS error");
        exit(1);
    }
    char *ip = inet_ntoa(*((struct in_addr *)h->h_addr));
    return ip;
}

int create_connection_socket(const char* ip, int port){
    int sockfd;
    struct sockaddr_in server_addr;
    
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        perror("error on socket()");
        return -1;
    }
    
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = inet_addr(ip);
    
    if(connect(sockfd, (struct sockaddr*) &server_addr, sizeof(server_addr)) < 0){
        perror("error on connect()");
        close(sockfd);
        return -1;
    }
    
    return sockfd;
}

int read_response_socket(int sockfd, char* response, int response_size){
    char buffer[1024];
    int total_bytes = 0;
    int bytes;
    
    memset(response, 0, response_size);
    
    while((bytes = read(sockfd, buffer, sizeof(buffer) - 1)) > 0){
        if(total_bytes + bytes < response_size - 1){
            memcpy(response + total_bytes, buffer, bytes);
            total_bytes += bytes;
            response[total_bytes] = '\0';
            
            if(bytes >= 2 && buffer[bytes-2] == '\r' && buffer[bytes-1] == '\n'){
                if(total_bytes >= 4 && response[3] == ' '){
                    break;
                }
            }
        } else {
            break;
        }
    }
    
    if(bytes < 0){
        perror("Error reading from socket");
        return -1;
    }
    
    printf("Server: %s", response);
    return total_bytes;
}

int send_command_socket(int sockfd, const char* command){
    printf("Client: %s", command);
    int bytes_sent = write(sockfd, command, strlen(command));
    if(bytes_sent < 0){
        perror("sending command error");
        return -1;
    }
    return bytes_sent;
}

int get_response_code(const char* response){
    if(strlen(response) >= 3){
        char code_str[4] = {0};
        strncpy(code_str, response, 3);
        return atoi(code_str);
    }
    return 0;
}

int ftp_login(int sockfd, const char* user, const char* password){
    char response[1024];
    char command[100];
    
    sprintf(command, "USER %s\r\n", user);
    if(send_command_socket(sockfd, command) < 0) return -1;
    
    if(read_response_socket(sockfd, response, sizeof(response)) < 0) return -1;
    
    int code = get_response_code(response);
    if(code != 331 && code != 230){
        printf("Error on command USER. Code: %d\n", code);
        return -1;
    }
    
    if(code == 331){
        sprintf(command, "PASS %s\r\n", password);
        if(send_command_socket(sockfd, command) < 0) return -1;
        
        if(read_response_socket(sockfd, response, sizeof(response)) < 0) return -1;
        
        code = get_response_code(response);
        if(code != 230){
            printf("Error in login. Code: %d\n", code);
            return -1;
        }
    }
    
    printf("Login successful!\n");
    return 0;
}

int passive_mode(int sockfd, char* data_ip, int* data_port){
    char response[1024];
    char command[] = "PASV\r\n";
    
    if(send_command_socket(sockfd, command) < 0) return -1;
    
    if(read_response_socket(sockfd, response, sizeof(response)) < 0) return -1;
    
    char* start = strchr(response, '(');
    char* end = strchr(response, ')');
    
    if(!start || !end){
        printf("Invalid PASV response format\n");
        return -1;
    }
    
    char pasv_data[100];
    int len = end - start - 1;
    strncpy(pasv_data, start + 1, len);
    pasv_data[len] = '\0';
    
    int h1, h2, h3, h4, p1, p2;
    if(sscanf(pasv_data, "%d,%d,%d,%d,%d,%d", &h1, &h2, &h3, &h4, &p1, &p2) != 6){
        printf("Error parsing PASV data\n");
        return -1;
    }
    
    sprintf(data_ip, "%d.%d.%d.%d", h1, h2, h3, h4);
    *data_port = p1 * 256 + p2;
    
    printf("Passive mode: IP = %s, Port = %d\n", data_ip, *data_port);
    return 0;
}

int change_directory(int sockfd, const char* path){
    if(strcmp(path, "/") == 0 || strlen(path) == 0){
        return 0;
    }
    
    char response[1024];
    char command[300];
    char dir_path[256];
    
    strcpy(dir_path, path);
    
    char* last_slash = strrchr(dir_path, '/');
    if(last_slash != NULL){
        if(*(last_slash + 1) != '\0'){
            *last_slash = '\0';
        }
    }
    
    if(strlen(dir_path) == 0 || strcmp(dir_path, "/") == 0){
        return 0;
    }
    
    sprintf(command, "CWD %s\r\n", dir_path);
    if(send_command_socket(sockfd, command) < 0) return -1;
    
    if(read_response_socket(sockfd, response, sizeof(response)) < 0) return -1;
    
    int code = get_response_code(response);
    if(code != 250){
        printf("Error changing directory. Code: %d\n", code);
        return -1;
    }
    
    return 0;
}

int download_file(int control_sock, int data_sock, const char* filename){
    char response[1024];
    char command[100];
    
    sprintf(command, "RETR %s\r\n", filename);
    if(send_command_socket(control_sock, command) < 0) return -1;
    
    if(read_response_socket(control_sock, response, sizeof(response)) < 0) return -1;
    
    int code = get_response_code(response);
    if(code != 150 && code != 125){
        printf("Error in RETR. Code: %d\n", code);
        return -1;
    }
    
    FILE* file = fopen(filename, "wb");
    if(!file){
        perror("Error creating file");
        return -1;
    }
    
    char buffer[4096];
    int bytes;
    
    while((bytes = read(data_sock, buffer, sizeof(buffer))) > 0){
        fwrite(buffer, 1, bytes, file);
    }
    
    if(bytes < 0){
        perror("Error reading data connection");
        fclose(file);
        return -1;
    }
    
    fclose(file);
    close(data_sock);
    
    if(read_response_socket(control_sock, response, sizeof(response)) < 0) return -1;
    
    code = get_response_code(response);
    if(code != 226){
        printf("Transfer failed. Code: %d\n", code);
        return -1;
    }
    
    printf("File '%s' downloaded successfully!\n", filename);
    return 0;
}

int main(int argc, char* argv[]){
    if(argc != 2){
        printf("Usage: %s ftp://[user:password@]host/path\n", argv[0]);
        return -1;
    }
    
    FTPInfo ftp_info;
    parse_ftp_url(argv[1], &ftp_info);
    
    printf("=== FTP Client ===\n");
    printf("User: %s\n", ftp_info.user);
    printf("Password: %s\n", ftp_info.password);
    printf("Host: %s\n", ftp_info.host);
    printf("Path: %s\n", ftp_info.path);
    printf("Filename: %s\n", ftp_info.filename);
    printf("==================\n");
    
    char* ip = get_ip(ftp_info.host);
    strcpy(ftp_info.ip, ip);
    printf("Server IP: %s\n", ftp_info.ip);
    
    int control_sock = create_connection_socket(ftp_info.ip, 21);
    if(control_sock < 0){
        printf("Failed to connect to FTP server\n");
        return -1;
    }
    
    char response[1024];
    
    if(read_response_socket(control_sock, response, sizeof(response)) < 0){
        close(control_sock);
        return -1;
    }
    
    if(ftp_login(control_sock, ftp_info.user, ftp_info.password) < 0){
        printf("Login failed\n");
        close(control_sock);
        return -1;
    }
    
    if(change_directory(control_sock, ftp_info.path) < 0){
        printf("Failed to change directory\n");
        close(control_sock);
        return -1;
    }
    
    char data_ip[16];
    int data_port;
    
    if(passive_mode(control_sock, data_ip, &data_port) < 0){
        printf("Failed to enter passive mode\n");
        close(control_sock);
        return -1;
    }
    
    int data_sock = socket(AF_INET, SOCK_STREAM, 0);
    if(data_sock < 0){
        perror("Failed to create data socket");
        close(control_sock);
        return -1;
    }
    
    struct sockaddr_in data_addr;
    memset(&data_addr, 0, sizeof(data_addr));
    data_addr.sin_family = AF_INET;
    data_addr.sin_port = htons(data_port);
    data_addr.sin_addr.s_addr = inet_addr(data_ip);
    
    if(connect(data_sock, (struct sockaddr*)&data_addr, sizeof(data_addr)) < 0){
        perror("Failed to connect to data port");
        close(data_sock);
        close(control_sock);
        return -1;
    }
    
    if(strlen(ftp_info.filename) > 0){
        if(download_file(control_sock, data_sock, ftp_info.filename) < 0){
            printf("File download failed\n");
            close(data_sock);
            close(control_sock);
            return -1;
        }
    } else {
        printf("Error: No filename specified in path\n");
        close(data_sock);
        close(control_sock);
        return -1;
    }
    
    send_command_socket(control_sock, "QUIT\r\n");
    read_response_socket(control_sock, response, sizeof(response));
    
    close(data_sock);
    close(control_sock);
    
    printf("Connection closed.\n");
    return 0;
}