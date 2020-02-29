#pragma once
//Server Details
#include "DataSync/serverdetails.hpp"


/* UDP client in the internet domain */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
// To use time library of C
#include "../Timing/timing.hpp"

//to use threading
#include <pthread.h>

//to use formation data structure
#include "../Data/formationdata.hpp"

//for protocol
#include "protocol/packet.hpp"

//for packet reception
#include "protocol/packetreceiver.hpp"

//for packet send queue
#include "clientpacketqueue.hpp"

//for handling packets
#include "clientpackethandler.hpp"

void error(const char *);
void delay(int milli_seconds)
{

    // Stroing start time
    clock_t start_time = clock();

    // looping till required time is not acheived
    while (clock() < start_time + milli_seconds)
        ;
}

void error(const char *msg)
{
    if (errno == EAGAIN || errno == EINTR)
    {
        //printf("%u", errno);
        return;
    }
    perror(msg);
    exit(0);
}

void *
start_client_thread(void *args);

extern std::function<void(const Packet &packet, void *object)> ProcessPacket;

template <typename data_type = double>
class Client
{
public:
    int sock, n;
    unsigned int length;
    struct sockaddr_in server, from;
    struct hostent *hp;
    char buffer[256], readBuffer[READ_BUFFER_SIZE];

private:
    bool turnOffThread = false;
    unsigned long int nextDataSendTime_ms = 0;
    pthread_t thread_id;
    bool thread_running = false;

    PacketReceiver packetReceiver;

    ClientPacketQueue clientPacketQueue;

    // {
    //     struct timeval currentTime;
    //     gettimeofday(&currentTime, NULL);
    //     unsigned long int currrenTime_ms = (currentTime.tv_sec * 1000 + currentTime.tv_usec / 1000);
    //     return currrenTime_ms;
    // }

    // inline bool SendDataToServer()
    // {

    //     unsigned long int currentDataSendTime_ms = GetCurrentTime_ms();
    //     //printf("currentDataSendTime_ms:%u\n", currentDataSendTime_ms);
    //     if (nextDataSendTime_ms < currentDataSendTime_ms)
    //     {
    //         //printf("currentDataSendTime_ms:%u\n", currentDataSendTime_ms);
    //         //printf("CLIENT_MSG_TIME_PERIOD_MS:%d\n", CLIENT_MSG_TIME_PERIOD_MS);
    //         nextDataSendTime_ms = currentDataSendTime_ms + (long unsigned int)CLIENT_MSG_TIME_PERIOD_MS;
    //         return true;
    //     }
    //     return false;
    // }

public:
    //FormationData reference
    FormationData<data_type> &formationData;
    ClientPacketHandler<data_type> clientPacketHandler;
    void
    ThreadExecute()
    {
        thread_running = true;
        while (!turnOffThread)
        {

            if (CheckDataSendTiming(nextDataSendTime_ms, CLIENT_MSG_TIME_PERIOD_MS))
            {
                
                bzero(buffer, 256);
                DroneData<data_type> droneData;
                formationData.GetCurrentDroneData(droneData);
                printf("Drone%d:x=%f,y=%f\n", droneData.droneID, droneData.droneStates[0], droneData.droneStates[1]);
               
                Packet packet(droneData);

                packet.GetByteStream(buffer, n);
                SendData();
            }
            else
            {

                usleep(50);
            }
            Packet packet;
            if (clientPacketQueue.GetPacket(packet))
            {
                packet.GetByteStream(buffer, n);
                printf("Sending ACK\n");
                SendData();
            }
            RecvData();
        }
        thread_running = false;
        printf("DataSync client thread exiting\n");
    }

    Client() : formationData(FormationData<data_type>::GetFormationData()), clientPacketHandler()
    {

        sock = socket(AF_INET, SOCK_DGRAM, 0);

        struct timeval read_timeout;
        read_timeout.tv_sec = 0;
        read_timeout.tv_usec = 10;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);

        if (sock < 0)
            error("socket");

        server.sin_family = AF_INET;
        hp = gethostbyname(ServerDetails::SERVER);
        if (hp == 0)
            error("Unknown host");

        bcopy((char *)hp->h_addr,
              (char *)&server.sin_addr,
              hp->h_length);
        server.sin_port = htons(atoi(ServerDetails::SERVER_PORT));
        length = sizeof(struct sockaddr_in);

        nextDataSendTime_ms = 0;

        thread_id = 0;
    }

    void SendData()
    {

        //printf("stringLength:%d\n",n);
        n = sendto(sock, buffer,
                   n, 0, (const struct sockaddr *)&server, length);
        if (n < 0)
            error("Sendto");
        //usleep(50);
    }

    void RecvData()
    {
        //printf("TryingToRecieve\n");
        int bytesRecieved = recvfrom(sock, readBuffer, READ_BUFFER_SIZE, 0, (struct sockaddr *)&from, &length);
        if (bytesRecieved > 0)
        {
            //printf("%u", bytesRecieved);
            packetReceiver.HandleDataWithPacketCallbacks(readBuffer, bytesRecieved, clientPacketHandler, clientPacketQueue);
        }
        else if (bytesRecieved == -1)
        {
            error("fromerror:");
        }
    }

    void Start()
    {
        if (!thread_running)
        {
            turnOffThread = false;
            printf("Starting DataSync Client Thread\n");
            int result = pthread_create(&thread_id, NULL, &start_client_thread, this);
            if (result)
                throw result;
        }
        else
        {
            printf("DataSync Client Thread already running\n");
        }
        printf("DataSync Client Thread started\n");
    }

    void Stop()
    {
        if (thread_running)
        {
            turnOffThread = true;
            pthread_join(thread_id, NULL);
            printf("stoped DataSync Client Thread\n");
        }
        else
        {
            printf("No DataSync Client Thread running to stop\n");
        }
    }

    ~Client()
    {
        if (!turnOffThread)
        {
            Stop();
        }

        close(sock);
    }
};

void *
start_client_thread(void *args)
{
    // takes an autopilot object argument
    Client<double> *client = (Client<double> *)args;

    client->ThreadExecute();

    // done!
    return NULL;
}


