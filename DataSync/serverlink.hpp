#pragma once
#include <sys/types.h>
#include <arpa/inet.h>

#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h>
#include <stdio.h>
#include <iostream>
#include <signal.h>

#include "serverdetails.hpp"
#include "../protocol/packet.hpp"

class ServerLink
{
    private:
        int sock, length, n;
        socklen_t addressLen;
        struct sockaddr_in server;
        struct sockaddr_in from;
        pthread_mutex_t lock;

        int InitializeDafaults()
        {
            if (pthread_mutex_init(&lock, NULL) != 0)
            {
                printf("\n mutex init failed\n");
                return 1;
            }
        }

        void error(const char *msg)
        {
            if(errno=EAGAIN)
                return ;
            printf("errno: %d", errno);
            perror(msg);
            exit(0);
    }

    
    public:
        ServerLink()
        {
            InitializeDafaults();

            sock = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock < 0)
                error("Opening socket");
            length = sizeof(server);
            bzero(&server, length);
            server.sin_family = AF_INET;
            server.sin_addr.s_addr = INADDR_ANY;
            server.sin_port = htons(atoi(ServerDetails::SERVER_PORT));

            struct timeval read_timeout;
            read_timeout.tv_sec = 0;
            read_timeout.tv_usec = 20;
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);

            if (bind(sock, (struct sockaddr *)&server, length) < 0)
                error("port binding failed");
            addressLen = sizeof(struct sockaddr_in);
        }
        ~ServerLink()
        {
            //printf("ServerLink distructor called\n");
            pthread_mutex_destroy(&lock);
            close(sock);
        }

        void CloseSocket()
        {
            close(sock);
        }

        int ReceiveData(char buffer[], struct sockaddr_in *fromAddress, socklen_t* addresslength)
        {
            pthread_mutex_lock(&lock);
            bzero(buffer, READ_BUFFER_SIZE);
            //std::cout << "size:" << dataHandler.Size() << std::endl;
            n = recvfrom(sock, buffer, READ_BUFFER_SIZE, 0, (struct sockaddr *)fromAddress, addresslength);
            if (n < 0){
                
                error("fromerror");
            }
            //usleep(50);
            pthread_mutex_unlock(&lock);

            return n;
        }

       
        int SendData(char buffer[], int length, struct sockaddr_in *toAddress, socklen_t addressLength)
        {
            pthread_mutex_lock(&lock);
            n = sendto(sock, buffer, length,
                       0, (struct sockaddr *)toAddress, addressLength);
            //usleep(50);
            pthread_mutex_unlock(&lock);
            if (n < 0){
                printf("%s\n", inet_ntoa(toAddress->sin_addr));
                error("sendto");
            }
            

            return n;
        }
};