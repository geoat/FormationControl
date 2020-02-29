#pragma once

#include <string>
#include <iostream>

#include "../Data/formationserverdata.hpp"
#include "../Data/communicationgraph.hpp"

#include "Types/formationtypes.hpp"
#include "serverlink.hpp"
#include "../Data/clientaddress.hpp"
#include "acknowledgedpacketqueue.hpp"

void *StartDataProcessingThread(void *args);
void *StartDispatcherThread(void *args);



class ServerDataHandler
{

    //Data Processor thread
    volatile bool turnOffDataProcessorThread = false;

    volatile bool dataProcessorThreadRunning = false;

    pthread_t data_processing_thread_id;

    

    //Data Dispatcher Thread
    volatile bool turnOffDispatcherThread = false;
    volatile bool dispatcherThreadRunning = false;
    pthread_t dispatcher_thread_id;
    
    //Communication Graph
    CommunicationGraph& communicationGraph;


    //used only by thread
    std::map<struct sockaddr_in, PacketReceiver> packetReceiverPool;

    
    //ServerLink
    ServerLink *serverLink;

    FormationServerDataTemp serverDataTemp;
    FormationServerData processedDroneData;

    //for acknowledged packet sending
    AckPacketQueue ackPacketQueue;

    //for configuration send timing
    _time_type nextConfigurationSendTime  = 0;

    PacketReceiver &
    GetPacketReceiver(const struct sockaddr_in &address)
    {
        auto search = packetReceiverPool.find(address);

        if (search == packetReceiverPool.end())
        {

            packetReceiverPool.emplace(address, PacketReceiver());
            search = packetReceiverPool.find(address);
        }
        return (search->second);
    }






public:
    ServerDataHandler() : communicationGraph(CommunicationGraph::GetCommunicationGraph())
    {

    }

    ~ServerDataHandler()
    {
        if (dispatcherThreadRunning | dataProcessorThreadRunning)
            Stop();
    }
    std::string GetDataToPrint()
    {
        return processedDroneData.GetDataToPrint();
    }



    void SendAckPacket(Packet &packet)
    {
        ackPacketQueue.InsertPacket(packet);
    }

    void ReportAck(Packet &packet)
    {
        ackPacketQueue.ReportAck(packet);
    }

    void ReportData(ServerSideDroneDataTemp &newData)
    {
        serverDataTemp.ReportData(newData);
    }

    int Size()
    {
        return serverDataTemp.Size();
    }

    void Start(ServerLink* serverLink_in)
    {
        serverLink = serverLink_in;

        if (!dataProcessorThreadRunning)
        {
            turnOffDataProcessorThread = false;
            printf("Starting Data Processing Thread\n");
            int result = pthread_create(&data_processing_thread_id, NULL, &StartDataProcessingThread, this);
            if (result)
                throw result;
        }
        else
        {
            printf("Data Processing Thread already running\n");
        }

        if (!dispatcherThreadRunning)
        {
            turnOffDispatcherThread = false;
            printf("Starting Dispatcher Thread\n");
            int result = pthread_create(&dispatcher_thread_id, NULL, &StartDispatcherThread, this);
            if (result)
                throw result;
        }
        else
        {
            printf("Dispatcher Thread already running\n");
        }
    }

    void Stop()
    {
        if (dataProcessorThreadRunning)
        {
            dataProcessorThreadRunning = false;
            turnOffDataProcessorThread = true;
            pthread_join(data_processing_thread_id, NULL);
            printf("stopped Data Processing Thread\n");
        }
        else
        {
            printf("No Data Processing Thread running to stop\n");
        }

        if (dispatcherThreadRunning)
        {
            dispatcherThreadRunning = false;
            turnOffDispatcherThread = true;
            pthread_join(dispatcher_thread_id, NULL);
            printf("stopped Dispatcher Thread\n");
        }
        else
        {
            printf("No Dispatcher Thread running to stop\n");
        }
    }

    void DataProcessorThreadExecute()
    {
        dataProcessorThreadRunning = true;
        ServerSideDroneDataTemp currentDroneDataTemp;

        while (!turnOffDataProcessorThread)
        {

            if (serverDataTemp.Size() > 0)
            {
                serverDataTemp.GetNextData(currentDroneDataTemp);

                PacketReceiver packetReceiver = GetPacketReceiver(currentDroneDataTemp.address);

                if (packetReceiver.HandleDataAndGetLastPacket(currentDroneDataTemp.buffer, currentDroneDataTemp.count))
                {
                    Packet packet = packetReceiver.currentPacket;
                    if (packet.type == T_DATA){
                        //printf("Drone%d:x=%f,y=%f\n", packet.droneID, ((float *)(packet.data))[0], ((float *)(packet.data))[1]);
                        //printf("Drone%d:C0=%f", packet.droneID,((float *)(packet.data))[12]);
                        processedDroneData.InsertOrAssignPacket(packet, currentDroneDataTemp.address, currentDroneDataTemp.addressLength);
                    }
                    else if (packet.type == T_ACK)
                    {
                        printf("acknowledged\n");
                        ackPacketQueue.ReportAck(packet);
                    }
                }
            }
            else
            {
                usleep(50);
            }
        }
    }

    void DispatcherThreadExecute()
    {

        dispatcherThreadRunning = true;

        char bufferSend[256];
        Packet packetToSend;
        std::vector<_droneID_type> targetDrones;
        std::vector<_droneID_type> sourceDrones;
        int streamLength;
        ClientAddress targetDroneAddress;
        while ((!turnOffDispatcherThread) & (serverLink != nullptr))
        {
            
            //get IDs of drones which should be send data as per the map.
            targetDrones = communicationGraph.GetTargetDrones();
            if (targetDrones.size() > 0)
            {
                
                //iterate through each eligible drone
                for (auto targetDrone : targetDrones)
                {
                    //printf("\ntarget:%d", targetDrone);
                    if (communicationGraph.TimingCheck(targetDrone))
                    {
                        //printf("\ntarget:%d", targetDrone);
                        //see whose packets should be send
                        sourceDrones = communicationGraph.GetSourceDrones(targetDrone);
                        if (!sourceDrones.empty())
                        {
                            for (auto sourceDrone : sourceDrones)
                            {
                                //printf(":source%d", sourceDrone);
                                //get the address of target drone
                                if (processedDroneData.GetDroneAddress(targetDrone, targetDroneAddress)){
                                    bzero(bufferSend, 256);
                                    //get the packet of the source drone
                                    if (processedDroneData.GetDronePacket(sourceDrone, packetToSend))
                                    {   //serialize packet                                 
                                        packetToSend.GetByteStream(bufferSend, streamLength);
                                        //send packet
                                        //printf("Trying to sendTo:%d\n", targetDrone);
                                        serverLink->SendData(bufferSend, streamLength, &targetDroneAddress.address,targetDroneAddress.addressLength);
                                        usleep(50);
                                        
                                    }
                                    // std::vector<Packet> ackPacketsToSend = ackPacketQueue.GetPacketsToSend(targetDrone);
                                    // if (!ackPacketsToSend.empty())
                                    // {
                                    //     for (auto &&ackPacket : ackPacketsToSend)
                                    //     {
                                    //         ackPacket.GetByteStream(bufferSend, streamLength);
                                    //         serverLink->SendData(bufferSend, streamLength, &targetDroneAddress.address, targetDroneAddress.addressLength);
                                    //         //usleep(50);
                                    //     }
                                        
                                    // }
                                }
                            }                           

                        }
                    }
                    
                }
                //usleep(100);
            }
            else
            {
                usleep(1000);
            }

            if (CheckDataSendTiming(nextConfigurationSendTime, CONFIGURATION_TO_CLIENT_TIMING))
            {
                //printf("reached:%d\n", nextConfigurationSendTime);
                auto dronesToSend = ackPacketQueue.GetDroneIDs();

                for (auto &&droneTo : dronesToSend)
                {
                    if (processedDroneData.GetDroneAddress(droneTo,targetDroneAddress))
                    {
                        std::vector<Packet> ackPacketsToSend = ackPacketQueue.GetPacketsToSend(droneTo);
                        if (!ackPacketsToSend.empty())
                        {
                            for (auto &&ackPacket : ackPacketsToSend)
                            {
                                bzero(bufferSend, 256);
                                ackPacket.GetByteStream(bufferSend, streamLength);
                                serverLink->SendData(bufferSend, streamLength, &targetDroneAddress.address, targetDroneAddress.addressLength);
                                usleep(5);
                            }
                        }
                    }
                    
                }
                
            }

        
        }
        //printf("While loop exited\n");
        turnOffDispatcherThread = true;
    }
};

void *StartDataProcessingThread(void *args)
{
    // takes an autopilot object argument
    ServerDataHandler *serverDataHandler = (ServerDataHandler *)args;

    serverDataHandler->DataProcessorThreadExecute();

    // done!
    return NULL;
}

void *StartDispatcherThread(void *args)
{
    // takes an autopilot object argument
    ServerDataHandler *serverDataHandler = (ServerDataHandler *)args;

    serverDataHandler->DispatcherThreadExecute();

    // done!
    return NULL;
}