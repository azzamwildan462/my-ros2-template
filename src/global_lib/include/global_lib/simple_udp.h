#ifndef SIMPLE_UDP_H
#define SIMPLE_UDP_H

#include <iostream>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#ifndef NO_ERROR
#define NO_ERROR 0
#endif

#ifndef ERROR
#define ERROR -1
#endif

class UDP
{
public:
    int sockfd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;

    char buffer[1024];
    socklen_t addr_len;

    int iam_as_a_server_port;
    int iam_as_a_client_server_port;
    const char *ip_address;

public:
    UDP()
    {
        sockfd = -1;
        addr_len = sizeof(client_addr);
        iam_as_a_server_port = 0;
        iam_as_a_client_server_port = 0;
        ip_address = nullptr;
    }

    ~UDP()
    {
        close();
    }

    int init_as_client()
    {
        if (ip_address == nullptr)
        {
            perror("IP address is not set.");
            return ERROR;
        }

        if (iam_as_a_client_server_port == 0)
        {
            perror("Client server port is not set.");
            return ERROR;
        }

        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0)
        {
            perror("socket");
            exit(EXIT_FAILURE);
        }

        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(iam_as_a_client_server_port);
        inet_pton(AF_INET, ip_address, &server_addr.sin_addr);

        return NO_ERROR;
    }

    int init_as_server()
    {
        if (iam_as_a_server_port == 0)
        {
            perror("Server port is not set.");
            return ERROR;
        }

        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0)
        {
            perror("socket");
            exit(EXIT_FAILURE);
        }

        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(iam_as_a_server_port);
        server_addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
            perror("bind");
            exit(EXIT_FAILURE);
        }

        return NO_ERROR;
    }

    int send(const char *data, size_t len)
    {
        ssize_t sent_bytes = sendto(sockfd, data, len, 0, (struct sockaddr *)&server_addr, sizeof(server_addr));
        if (sent_bytes < 0)
        {
            perror("sendto");
            return ERROR;
        }
        return NO_ERROR;
    }

    int receive(char *buffer, size_t len, int flags = 0)
    {
        addr_len = sizeof(client_addr);
        ssize_t received_bytes = recvfrom(sockfd, buffer, len, flags, (struct sockaddr *)&client_addr, &addr_len);
        if (received_bytes < 0)
        {
            perror("recvfrom");
            return ERROR;
        }
        return received_bytes;
    }

    void close()
    {
        if (sockfd >= 0)
        {
            ::close(sockfd);
            sockfd = -1;
        }
    }
};

#endif