#pragma once
//to store address datatypes;
#include <netinet/in.h>
#include <map>

#include "formationconfig.hpp"

#include "protocol/packet.hpp"

#include "pthread.h"
#include "utility"
#include "Types/formationtypes.hpp"
#include "protocol/packetreceiver.hpp"

#include "formationserverdatatemp.hpp"
#include "stdio.h"
#include <iostream>
#include "clientaddress.hpp"
#include "../Types/formationtypes.hpp"
#include <tuple>
#include "../Timing/timing.hpp"

#include <string>
class FormationServerData
{

    std::map<_droneID_type, std::tuple<Packet, ClientAddress, _time_type>> droneDataMatrix;
    pthread_mutex_t lock;

    int InitializeDafaults()
    {
        if (pthread_mutex_init(&lock, NULL) != 0)
        {
            printf("\n mutex init failed\n");
            return 1;
        }
    }

public:
    FormationServerData()
    {
        InitializeDafaults();
    }

    ~FormationServerData()
    {
        pthread_mutex_destroy(&lock);
    }

    std::string GetDataToPrint()
    {
        using namespace std;
        string output = "";
        pthread_mutex_lock(&lock);
        
        
        for (auto &&drone : droneDataMatrix)
        {
            //std::cout<<"Reached"<<std::endl;
            output += "Drone" + std::to_string(drone.first);
            output += ",Time_ms=" + std::to_string(std::get<2>(drone.second));
            Packet &packet = std::get<0>(drone.second);
            output += ",x=" + std::to_string(((float *)(packet.data))[0]) + ",y=" + std::to_string(((float *)(packet.data))[1]) + ",h=" + std::to_string(((float *)(packet.data))[2]);
            output += ",Fx=" + std::to_string(((float *)(packet.data))[12]);
            output += ",Fy=" + std::to_string(((float *)(packet.data))[13]);
            output += ",Fz=" + std::to_string(((float *)(packet.data))[14]);
            output += ",Mx=" + std::to_string(((float *)(packet.data))[15]);
            output += ",My=" + std::to_string(((float *)(packet.data))[16]);
            output += ",Mz=" + std::to_string(((float *)(packet.data))[17]);
            output+="\n";
        }
        pthread_mutex_unlock(&lock);
        return output;
        }

    void
    InsertOrAssignPacket(Packet packet, struct sockaddr_in address, socklen_t addressLength)
    {
        pthread_mutex_lock(&lock);
        auto search = droneDataMatrix.find(packet.droneID);

        if (search == droneDataMatrix.end())
        {
            ClientAddress clientAddress;
            clientAddress.address = address;
            clientAddress.addressLength = addressLength;
            _time_type lastRecievedTime = GetCurrentTime_ms();
            auto data = std::make_tuple(packet, clientAddress, lastRecievedTime);
            // std::pair<Packet, ClientAddress> data;
            // data.first = packet;
            // data.second.address = address;
            // data.second.addressLength = addressLength;

            droneDataMatrix.emplace(packet.droneID, data);
        }
        else
        {
            std::get<0>(search->second) = packet;
            std::get<1>(search->second).address = address;
            std::get<1>(search->second).addressLength = addressLength;
            std::get<2>(search->second) = GetCurrentTime_ms();
            //search->second.first = packet;
            //search->second.second.address = address;
            //search->second.second.addressLength = addressLength;

        }
        
        pthread_mutex_unlock(&lock);
        //printf("InsertOrAssignPacket:Count:%d\n", droneDataMatrix.size());
    }

    bool GetDroneAddress(const _droneID_type &droneID, ClientAddress& clientAddress)
    {
        bool addressAvailable = false;;
        pthread_mutex_lock(&lock);
        auto search = droneDataMatrix.find(droneID);

        if (search != droneDataMatrix.end())
        {
            clientAddress = std::get<1>(search->second);
                //clientAddress = search->second.second;
           
                addressAvailable = true;
        }
        pthread_mutex_unlock(&lock);
        return addressAvailable;
    }

    bool GetDronePacket(const _droneID_type& droneID, Packet &dronePacket)
    {
        bool packetAvailable = false;
        pthread_mutex_lock(&lock);
        auto search = droneDataMatrix.find(droneID);

        if (search != droneDataMatrix.end())
        {
            dronePacket = std::get<0>(search->second);
            //dronePacket = search->second.first;
            if (std::get<2>(search->second) < (GetCurrentTime_ms() + CLIENT_MSG_CONNECTION_LOST_TIME_PERIOD_MS))
                packetAvailable = true;
        }
        pthread_mutex_unlock(&lock);
        return packetAvailable;
    }
};
