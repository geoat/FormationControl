#pragma once

#include <map>
#include <vector>



#include "protocol/packet.hpp"

#include "pthread.h"
#include "utility"
#include "Types/formationtypes.hpp"



#include "stdio.h"



#include "../Timing/timing.hpp"
#include "../protocol/byteconverter.hpp"

struct PacketKey
{

    public : 
    typedef unsigned int KeyType;
    static KeyType GetKey(unsigned char packetType)
    {
        static Converter<KeyType> converter;
        KeyType value = 0;
        bzero(converter.m_bytes,4);
        switch (packetType)
        {
            case T_CONFIG_KPKV:

                converter.m_bytes[0] = T_CONFIG_KPKV;
                value = converter.m_value;
                break;


            case T_CONFIG_SGAIN:
                converter.m_bytes[0] = T_CONFIG_SGAIN;
                value = converter.m_value;
            break;

            case T_CONFIG_PGAIN:
                converter.m_bytes[0] = T_CONFIG_PGAIN;
                value = converter.m_value;
                break;

            case T_CONFIG_COMM_GRAPH:
                converter.m_bytes[0] = T_CONFIG_COMM_GRAPH;
                value = converter.m_value;
                break;

            case T_MODE:
                converter.m_bytes[0] = T_MODE;
                value = converter.m_value;
            break;

            case T_EXIT:
                converter.m_bytes[0] = T_EXIT;
                value = converter.m_value;
                break;

            default:
            break;
        }
        return value;
    }
        
};

struct AckPacket
{
    Packet packet;
    bool acknowledged = false;

};



class AckPacketQueue
{
    std::map<_droneID_type, std::map<PacketKey::KeyType,AckPacket>> ackPacketQueueMatrix;
    pthread_mutex_t lock;
    public:
        AckPacketQueue()
        {
            if (pthread_mutex_init(&lock, NULL) != 0)
            {
                printf("\n AckPacketQueueMatrix mutex init failed\n");
                exit(0);
            }
    }
    ~AckPacketQueue()
    {
        pthread_mutex_destroy(&lock);
    }

    std::vector<_droneID_type> GetDroneIDs()
    {
        std::vector<_droneID_type> result;
        pthread_mutex_lock(&lock);
        result.reserve(ackPacketQueueMatrix.size());
        for (auto &&entry : ackPacketQueueMatrix)
        {
            result.push_back(entry.first);
        }
        pthread_mutex_unlock(&lock);
        return result;
    }

    std::vector < Packet> GetPacketsToSend(_droneID_type& droneID)
    {
        std::vector < Packet> packetsToSend;
        pthread_mutex_lock(&lock);
        auto search = ackPacketQueueMatrix.find(droneID);
        if (search != ackPacketQueueMatrix.end() && !search->second.empty())
        {
            auto& entries = search->second;
            packetsToSend.reserve(entries.size());
            for (auto &&entry : entries)
            {
                packetsToSend.push_back(entry.second.packet);
            }
           
        }
        pthread_mutex_unlock(&lock);
        return packetsToSend;
    }

    void InsertPacket(Packet& packet)
    {
        pthread_mutex_lock(&lock);

        auto searchQueue = ackPacketQueueMatrix.find(packet.droneID);
        if (searchQueue != ackPacketQueueMatrix.end())
        {
            PacketKey::KeyType searchKey = PacketKey::GetKey(packet.type);
            auto searchEntry = searchQueue->second.find(searchKey);
            if (searchEntry != searchQueue->second.end())
            {
                AckPacket ackPacket;
                ackPacket.packet = packet;
                ackPacket.acknowledged = false;
                searchEntry->second = ackPacket;
                //printf("chucking\n");
            }
            else
            {
                AckPacket ackPacket;
                ackPacket.packet = packet;
                ackPacket.acknowledged = false;
                searchQueue->second.emplace(searchKey, ackPacket);
            }
        }
        else
        {
            std::map<PacketKey::KeyType,AckPacket> queue;
            AckPacket ackPacket;
            ackPacket.packet = packet;
            ackPacket.acknowledged = false;
            queue.emplace(PacketKey::GetKey(packet.type), ackPacket);
            ackPacketQueueMatrix.emplace(packet.droneID, queue);
        }

        //printf("%d\n",ackPacketQueueMatrix.size());
        pthread_mutex_unlock(&lock);
    }

    void ReportAck(Packet packet)
    {
        pthread_mutex_lock(&lock);
        auto searchQueue = ackPacketQueueMatrix.find(packet.droneID);
        if (searchQueue != ackPacketQueueMatrix.end())
        {
            PacketKey::KeyType searchKey = PacketKey::GetKey(packet.data[0]);
            auto searchEntry = searchQueue->second.find(searchKey);
            if (searchEntry != searchQueue->second.end())
            {
                //printf("%d\n", searchQueue->second.size());
                if ((searchEntry->second.packet.CRC[0] == packet.data[1]) && (searchEntry->second.packet.CRC[1] == packet.data[2])){
                    //printf("erased\n");
                    searchQueue->second.erase(searchEntry);

                }
                //printf("%d\n", searchQueue->second.size());
            }
        }
        
        pthread_mutex_unlock(&lock);
    }
};