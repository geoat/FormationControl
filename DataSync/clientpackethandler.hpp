#pragma once
#include "../Data/formationdata.hpp"
#include "../Data/dronedata.hpp"
#include "../protocol/packet.hpp"
#include "../protocol/byteconverter.hpp"
#include "clientpackethandler.hpp"
#include <cstring>
#include "../Data/referencemodel.hpp"
#include "../Data/autopilotdata.hpp"
#include "../Data/autopilotstates.hpp"
#include "../Data/estimationgains.hpp"
#include "../Data/pathplannerdata.hpp"

#include <iostream>

template <typename data_type = double>
class ClientPacketHandler
{
    AutoPilotData<data_type>& autoPilotData = AutoPilotData<data_type>::GetAutoPilotData();
    FormationData<data_type> &formationData;
    union Converter<Packet::data_type> converter;
    DroneData<data_type>
    GetDroneData(Packet &packet)
    {
        DroneData<data_type> droneData;
        
        droneData.droneID = packet.droneID;
        for (char i = 0; i < 12; i++)
        { //take care of endianess
            memcpy(converter.m_bytes, packet.data + (i * 4),4);
            //     converter.m_bytes[0] = packet.data[i * 4];
            // converter.m_bytes[1] = packet.data[i * 4 + 1];
            // converter.m_bytes[2] = packet.data[i * 4 + 2];
            // converter.m_bytes[3] = packet.data[i * 4 + 3];
            droneData.droneStates[i] = (data_type)converter.m_value;
            //droneData.droneStates.push_back((data_type)converter.m_value);
            //printf("droneData.droneStates[%d]:%f\n", i, droneData.droneStates[i]);
        }
        for (char i = 12; i < 18; i++)
        { //take care of endianess
            memcpy(converter.m_bytes, packet.data + (i * 4), 4);
            //     converter.m_bytes[0] = packet.data[i * 4];
            // converter.m_bytes[1] = packet.data[i * 4 + 1];
            // converter.m_bytes[2] = packet.data[i * 4 + 2];
            // converter.m_bytes[3] = packet.data[i * 4 + 3];
            //droneData.controlInput.push_back((data_type)converter.m_value);
            droneData.controlInput[i-12] = (data_type)converter.m_value;
            //printf("droneData.controlInput[%d]:%f\n", i-12, droneData.controlInput[i-12]);
        }

        return droneData;
    }

public:
    virtual void operator()(Packet &packet, ClientPacketQueue& clientPacketQueue)
    {
        //printf("reached\n");
        if (packet.type == T_DATA){
            //printf("data received\n");
            DroneData<data_type> droneDataRecieved = GetDroneData(packet);
            formationData.UpdateDroneData(droneDataRecieved);
            printf("Drone%d,x=%f,y=%f,Time=%u\n", droneDataRecieved.droneID, droneDataRecieved.droneStates[0], droneDataRecieved.droneStates[1], droneDataRecieved.lastReportedTime);
        }else if(packet.type == T_CONFIG_KPKV)
        {
            printf("KpKv config received\n");
            

            //register Kp Kv values;
            ReferenceModel<data_type>::GetReferenceModel().SetKpKv(packet);
            
            //acknowledge reception
            Packet ackPacket;
            ackPacket.CreateAckPacket(packet);
            clientPacketQueue.InsertPacketToSend(ackPacket);
        }

        else if(packet.type == T_MODE)
        {
            printf("mode updated\n");
            autoPilotData.SetAutoPilotState((AutoPilotStates)packet.data[0]);
            Packet ackPacket;
            ackPacket.CreateAckPacket(packet);
            clientPacketQueue.InsertPacketToSend(ackPacket);
        }
        else if (packet.type == T_EXIT)
        {
            printf("Exit received\n");
            autoPilotData.SetExitProgram();
            Packet ackPacket;
            ackPacket.CreateAckPacket(packet);
            clientPacketQueue.InsertPacketToSend(ackPacket);
        }
        else if (packet.type == T_CONFIG_SGAIN)
        {
             printf("SGain config received\n");
             FormationEstimationGains<data_type>::GetFormationEstimationGains().SetS(packet);
             Packet ackPacket;
             ackPacket.CreateAckPacket(packet);
             clientPacketQueue.InsertPacketToSend(ackPacket);
        }
        else if (packet.type == T_CONFIG_PGAIN)
        {
            printf("PGain config received\n");
            FormationEstimationGains<data_type>::GetFormationEstimationGains().SetP(packet);
            std::cout<<FormationEstimationGains<data_type>::GetFormationEstimationGains().GetP()<<std::endl;
            Packet ackPacket;
            ackPacket.CreateAckPacket(packet);
            clientPacketQueue.InsertPacketToSend(ackPacket);

        }
        else if (packet.type == T_CONFIG_COMM_GRAPH)
        {
            printf("Communication graph config received\n");
            printf("Setting to Type %d\n", (NodeType)packet.data[0]);
            autoPilotData.SetNodeType((NodeType)packet.data[0]);
            formationData.SetSourceDrones(packet.data + 1, packet.dataLength - 1);

        
            Packet ackPacket;
            ackPacket.CreateAckPacket(packet);
            clientPacketQueue.InsertPacketToSend(ackPacket);
        }
        else if(packet.type == T_PATHTYPE)
        {
            printf("path type config received\n");
            PathPlannerData<data_type> newPathPlannerData;
            if (GetPathPlannerData(newPathPlannerData, packet))
            {
                
                formationData.ReportNewPathPlannerData(newPathPlannerData);
            }
            Packet ackPacket;
            ackPacket.CreateAckPacket(packet);
            clientPacketQueue.InsertPacketToSend(ackPacket);
        }
    }

    bool GetPathPlannerData(PathPlannerData<data_type>& inputPathPlannerData, const Packet &packet)
    {
        bool success = false;
        if (packet.type == T_PATHTYPE)
        {
            inputPathPlannerData.pathType = (PathType)packet.data[0];
            auto otherDatas = packet.data + 1;
            switch (inputPathPlannerData.pathType)
            {
            case PathType::Line:
            {
                auto sizeOfDataSend = sizeof(Packet::data_type);
                memcpy(converter.m_bytes, otherDatas, sizeOfDataSend);
                inputPathPlannerData.lineOrigin.x = converter.m_value;
                memcpy(converter.m_bytes, otherDatas + sizeOfDataSend, sizeOfDataSend);
                inputPathPlannerData.lineOrigin.y = converter.m_value;
                memcpy(converter.m_bytes, otherDatas + sizeOfDataSend * 2, sizeOfDataSend);
                inputPathPlannerData.lineSlope.x = converter.m_value;
                memcpy(converter.m_bytes, otherDatas + sizeOfDataSend * 3, sizeOfDataSend);
                inputPathPlannerData.lineSlope.y = converter.m_value;

                success = true;
            }
            break;

            case PathType::Orbit:
            {
                auto sizeOfDataSend = sizeof(Packet::data_type);
                memcpy(converter.m_bytes, otherDatas, sizeOfDataSend);
                inputPathPlannerData.radius = converter.m_value;
                memcpy(converter.m_bytes, otherDatas + sizeOfDataSend, sizeOfDataSend);
                inputPathPlannerData.orbitCentre.x = converter.m_value;
                memcpy(converter.m_bytes, otherDatas + sizeOfDataSend * 2, sizeOfDataSend);
                inputPathPlannerData.orbitCentre.y = converter.m_value;
                success = true;
            }
            break;

            case PathType::WayPoints:
            {
                auto sizeOfDataSend = sizeof(Packet::data_type);
                memcpy(converter.m_bytes, otherDatas, sizeOfDataSend);
                inputPathPlannerData.radius = converter.m_value;
                std::cout << "Radius:"<<inputPathPlannerData.radius << std::endl;
                std::cout << "WayPoints:"<< std::endl;
                for (size_t i = sizeOfDataSend + 1; i < packet.dataLength; i = i + 2 * sizeOfDataSend)
                {
                    // std::cout<<i<<std::endl;
                    // std::cout << (int)packet.dataLength << std::endl;
                    Point<data_type> point;
                    memcpy(converter.m_bytes, packet.data + i, sizeOfDataSend);
                    point.x = converter.m_value;
                    // std::cout << i + sizeOfDataSend << std::endl;
                    memcpy(converter.m_bytes, packet.data + i + sizeOfDataSend, sizeOfDataSend);
                    point.y = converter.m_value;
                    inputPathPlannerData.wayPoints.push_back(point);
                    std::cout <<"x:"<< point.x << ",y:" << point.y << std::endl;
                }
                success = true;

            }
            break;

            default:

            break;
            }
        }
        return success;
    }

    ClientPacketHandler() : formationData(FormationData<data_type>::GetFormationData())
    {

    }
};