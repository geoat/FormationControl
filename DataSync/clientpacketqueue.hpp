#pragma once
#include "protocol/packet.hpp"
#include <pthread.h>
#include <stdio.h>

class ClientPacketQueue
{
    std::vector<Packet> packetQueue;
    pthread_mutex_t lock;

public:
    ClientPacketQueue()
    {
        packetQueue.reserve(4);
        if (pthread_mutex_init(&lock, NULL) != 0)
        {
            printf("\n ClientPacketQueue mutex init failed\n");
            exit(0);
        }
    }
    ~ClientPacketQueue() 
    {
        pthread_mutex_destroy(&lock);
    }

    void InsertPacketToSend(const Packet& packet)
    {
        pthread_mutex_lock(&lock);
        packetQueue.push_back(packet);
        //printf("firstqueueCount:%d\n",packetQueue.size());
        pthread_mutex_unlock(&lock);
    }
    bool GetPacket(Packet& packet)
    {
        
        bool result = false;
        pthread_mutex_lock(&lock);
        result = !packetQueue.empty();
        //printf("secondqueueCount:%d\n", packetQueue.size());
        if (result)
        {
            //printf("\n Elements in queue\n");
            packet = packetQueue.front();
            packetQueue.erase(packetQueue.begin());
            
        }
        pthread_mutex_unlock(&lock);
        return result;
    }
};
