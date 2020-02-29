#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h>
#include <stdio.h>
#include <iostream>
#include <signal.h>

#include "termiosextn.hpp"

#include "DataSync/serverdetails.hpp"
#include "protocol/packet.hpp"
#include "Data/formationserverdata.hpp"
#include "DataSync/serverdatahandler.hpp"
#include "DataSync/serverlink.hpp"

#include "Data/autopilotstates.hpp"

//for handling configuration
#include "Data/configuration.hpp"

//timing
#include "Timing/timing.hpp"

#include "Types/formationtypes.hpp"

#include "Data/nodetype.hpp"

//for printing
#define clearUI() printf("\033[H\033[J")
#define setCursor(x, y) printf("\033[%d;%dH", x, y)

//for writing data log
#include <string>
#include <fstream>

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

void quit_handler(int sig);
typedef double data_type;
Configuration<data_type> configuration;
ServerDataHandler *dataHandler_Quit;
ServerLink *serverLink_quit;
volatile bool run = true;
volatile bool syncConfigurationWithDrones = false;
void StartConfiguration();
void SyncConfiguration(ServerDataHandler &serverDataHandler);
void kb_input_handler(unsigned char c, ServerDataHandler &serverDataHandler);

//output file stream
std::ofstream oFile;

int
main()
{

    //open oFile
    oFile.open("serverout.csv");

    oFile<<"Drone,Time_ms,x,y,Fx,Fy,Fz,Mx,My,Mz" << std::endl;

    configuration.load("formation.config");
    //std::cout << configuration.P << std::endl;
    configuration.save("formation.config");
    termios_puts("\nUAV Formation - Server Program\n");

    termios_initialize();
    // int sock, length, n;
    // socklen_t fromlen;
    // struct sockaddr_in server;
    // struct sockaddr_in from;
    // char buf[1024];

    // sock = socket(AF_INET, SOCK_DGRAM, 0);
    // if (sock < 0)
    //     error("Opening socket");
    // length = sizeof(server);
    // bzero(&server, length);
    // server.sin_family = AF_INET;
    // server.sin_addr.s_addr = INADDR_ANY;
    // server.sin_port = htons(atoi(ServerDetails::SERVER_PORT));

    // struct timeval read_timeout;
    // read_timeout.tv_sec = 0;
    // read_timeout.tv_usec = 10;
    // setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);

    // if (bind(sock, (struct sockaddr *)&server, length) < 0)
    //     error("port binding failed");
    // fromlen = sizeof(struct sockaddr_in);

    ServerLink serverLink;
    serverLink_quit = &serverLink;
    _droneID_type droneIDs[] = {2, 3};
    bool AdjacencyMatrix[][2] = {{0,1},{0,0}};
    CommunicationGraph& communicationGraph = CommunicationGraph::GetCommunicationGraph();
    communicationGraph.SetGraph(droneIDs, 2, AdjacencyMatrix);
    ServerDataHandler dataHandler;
    communicationGraph.SetGraph(configuration);
    dataHandler_Quit = &dataHandler;
    dataHandler.Start(&serverLink);
    ServerSideDroneDataTemp droneReadData;
    signal(SIGINT, quit_handler);
    
    //for reading keyboard
    int c = 0;

    StartConfiguration();

    //for display timing
    _time_type nextPrintTime_ms = 0;

    //sleep(1);
    while (run)
    {
        _time_type currentTime_ms = GetCurrentTime_ms();
        //printf("server\n");
        // bzero(droneReadData.buffer,READ_BUFFER_SIZE);
        // //std::cout << "size:" << dataHandler.Size() << std::endl;
        // n = recvfrom(sock, droneReadData.buffer, READ_BUFFER_SIZE, 0, (struct sockaddr *)&droneReadData.address, &fromlen);
        
        //Read date from the data buffer which is being filled using the data send by the drones
        int n = serverLink.ReceiveData(droneReadData.buffer, &(droneReadData.address),&(droneReadData.addressLength));
        //std::cout << n << std::endl;
        if (n>0)
        {
            
            droneReadData.count = n;
            dataHandler.ReportData(droneReadData);
            //std::cout << "data:" << droneReadData.buffer << std::endl;
            //serverLink.SendData(droneReadData.buffer, 20, &(droneReadData.address), droneReadData.addressLength);
        }
        if ((c = termios_getchar_nb()) != -1)
        {
            kb_input_handler(c, dataHandler);
        }

        usleep(50);
        SyncConfiguration(dataHandler);



        if (nextPrintTime_ms < currentTime_ms)
        {
            nextPrintTime_ms = currentTime_ms + 100;
            clearUI();
            setCursor(7, 0);
            static std::string dataString= "";
            dataString = dataHandler.GetDataToPrint();
            std::cout <<dataString;
            oFile << dataString.c_str();
        }
    }
    dataHandler.Stop();
    //write back configuration
    configuration.save("formation.config");
    clearUI();
    termios_close();
    oFile.close();
    return 0;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler(int sig)
{
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    //stop DataSync Client
    dataHandler_Quit->Stop();

    serverLink_quit->CloseSocket();

    signal(SIGALRM, SIG_IGN);
    termios_close();

    //close opne stream file
    oFile.close();
    clearUI();
    // end program here
    exit(0);
}
inline void ActivateDroneFromation(ServerDataHandler &serverDataHandler);
inline void DeactivateDroneFromation(ServerDataHandler &serverDataHandler);
inline void ExitDrones(ServerDataHandler &serverDataHandler);
inline void SendPathInformationToLeaders(ServerDataHandler &serverDataHandler);
void kb_input_handler(unsigned char c, ServerDataHandler &serverDataHandler)
{

    switch (c)
    {
    
    case 'f':
        ActivateDroneFromation(serverDataHandler);
        break;

    case 'q':
        DeactivateDroneFromation(serverDataHandler);
    break;

    case 'z':
        StartConfiguration();
        break;

    case 'e':
        setCursor(10, 0);
        std::cout << "Exiting Drones"<< std::endl;
        ExitDrones(serverDataHandler);
        break;

    case 'c':
        setCursor(10, 0);
        std::cout << "Loading Configuration" << std::endl;
        configuration.load("formation.config");
        StartConfiguration();
        break;

    case 27:
        //storeUIMessage("%i\n",c );
        if ((c = termios_getchar_nb()) != -1)
        {
            switch (c = termios_getchar_nb())
            {
            case 'C': //Right Arrow - RollDown


                break;
            case 'D': //Left Arrow - RollUp


                break;
            case 'B': //Arrow Down - PitchUP


                break;
            case 'A': //Up Arrow - PitchDown


                break;
            default:

                run = false;
                //ExitDrones(serverDataHandler);

                break;
            }
        }
        break;

        //case 27:
        //break;
    default:

        break;
    }
}
void StartConfiguration()
{
    syncConfigurationWithDrones = true;
}
void SyncConfiguration(ServerDataHandler& serverDataHandler)
{

    if (syncConfigurationWithDrones){
        for (const auto& drone : configuration.dronesAndTypes)
        {
            
            Packet packet;
            packet.CreateKpKvPacket(configuration, drone.first);
            //std::cout << "packetdroneID:" << drone.first << std::endl;
            serverDataHandler.SendAckPacket(packet);

            packet.CreateSGainPacket(configuration, drone.first);
            //std::cout << "packetdroneID:" << drone.first << std::endl;
            serverDataHandler.SendAckPacket(packet);

            packet.CreatePGainPacket(configuration, drone.first);
            //std::cout << "packetdroneID:" << drone.first << std::endl;
            serverDataHandler.SendAckPacket(packet);

            packet.CreateCommGraphPacket(configuration, drone.first);
            //std::cout << "packetdroneID:" << drone.first << std::endl;
            serverDataHandler.SendAckPacket(packet);

            // Packet packetAck;
            // packetAck.startByte = START_BYTE;
            // packetAck.droneID = drone.first;
            // packetAck.type = T_ACK;
            // packetAck.data[0] = packet.type;
            // packetAck.data[1] = CONFIG_TYPE_KPKV;
            // packetAck.data[2] = packet.CRC[0];
            // packetAck.data[3] = packet.CRC[1];
            // serverDataHandler.ReportAck(packetAck);
        }

        //send path information to path planners
        SendPathInformationToLeaders(serverDataHandler);

        syncConfigurationWithDrones = false;
    }   

    return;
}

inline void SendPathInformationToLeaders(ServerDataHandler &serverDataHandler)
{
    auto& pathPlannerData = configuration.pathPlannerData;

    Packet packet;
    packet.CreatePathInformationPacket(pathPlannerData, 0);
    
    for (const auto &drone : configuration.dronesAndTypes)
    {
        if((NodeType)drone.second.first == NodeType::PathPlanner)
        {
            //set drone id for target packet
            packet.droneID = drone.first;
            serverDataHandler.SendAckPacket(packet);
        }
    }
}

inline void ActivateDroneFromation(ServerDataHandler &serverDataHandler)
{
    for (const auto &drone : configuration.dronesAndTypes)
    {
        setCursor(10, 0);
        std::cout << "ActivatingDrone:" << drone.first << std::endl;
        Packet packet;
        packet.CreateModePacket(drone.first,AutoPilotStates::active);
        //std::cout << "packetdroneID:" << drone.first << std::endl;
        serverDataHandler.SendAckPacket(packet);
    }
    return;
}

inline void DeactivateDroneFromation(ServerDataHandler &serverDataHandler)
{
    for (const auto &drone : configuration.dronesAndTypes)
    {
        setCursor(10, 0);
        std::cout << "DeactivatingDrone:" << drone.first << std::endl;
        Packet packet;
        packet.CreateModePacket(drone.first, AutoPilotStates::safe);
        //std::cout << "packetdroneID:" << drone.first << std::endl;
        serverDataHandler.SendAckPacket(packet);
    }
    return;
}

inline void ExitDrones(ServerDataHandler &serverDataHandler)
{ 
    for (const auto &drone : configuration.dronesAndTypes)
    {
        Packet packet;
        packet.CreateExitPacket(drone.first);
        //std::cout << "packetdroneID:" << drone.first << std::endl;
        serverDataHandler.SendAckPacket(packet);
    }
    return;

}