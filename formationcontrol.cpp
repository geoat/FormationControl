#define SamplingTime 0.0025
#define SamplingTimeus (SamplingTime*1000000)
#define ControlTime (SamplingTime*10)
#define ControlTimeus (ControlTime*1000000)

//#define HITL

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
//for timer
#include <sys/time.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>

//for smart pointer
#include <memory>

//for nodes
#include "Data/nodetype.hpp"
#include "node.hpp"
#include "Data/formationdata.hpp"

//for mavlink
#include "mavlink_control.h"

//for DataSync
#include "DataSync/client.hpp"

//for autopilot data
#include "Data/autopilotdata.hpp"
#include <cstdlib>

//timing
#include "Timing/timing.hpp"
//for general types used in formation control
#include "Types/formationtypes.hpp"

//for reading and writing files
#include <fstream>

//for accesing/configuring server details
#include "DataSync/serverdetails.hpp"

//variables for mavlink connection
char *uart_name;
int baudrate;

typedef double data_type;

//autopilot data reference
AutoPilotData<data_type> &autoPilotData = AutoPilotData<data_type>::GetAutoPilotData();

//Formation ControlData
FormationData< data_type> &formationData = FormationData< data_type>::GetFormationData();

//for stoping DataSync Client Safely;
Client<data_type>* dataSyncClient_Quit;


// --------------------------------------------------------------------------
//   CONFIGURE TIMER
// --------------------------------------------------------------------------
struct sigaction sa;
struct itimerval timer;


//forward declarations
void
InitializeMavlinkConnection(int argc, char **argv);
void 
InitializeTimer();
#ifdef HITL
void 
UpdateCurrentDroneData(const Mavlink_Messages messages);
#else
void UpdateCurrentDroneData(std::unique_ptr<Node<data_type>> &node);
#endif

//Pointer which decides the type of Node - PathPlanner/Leader/Follower
std::unique_ptr<Node<data_type>> node;

void ChangeType(NodeType type)
{
    switch (type)
    {
    case NodeType::PathPlanner:
        node->ReleaseType();
        printf("Switching to PathPlanner\n");
        node = std::unique_ptr<PathPlanner<data_type>>(new PathPlanner<data_type>(SamplingTime, ControlTime));

        break;
    case NodeType::Leader:
        node->ReleaseType();
        printf("Switching to Leader Type\n");
        node = std::unique_ptr<LeaderNode<data_type>>(new LeaderNode<data_type>(formationData, SamplingTime, ControlTime));
        break;
    case NodeType::Follower:
         printf("Switching to Follower Type\n");
         node = std::unique_ptr<FollowerNode<data_type>>(new FollowerNode<data_type>(formationData, SamplingTime, ControlTime));
        break;
    default:
        break;
    }
}

int main(int argc, char **argv)
{
    /*4 threads
      - Client for recieving Data and sending data
      - main thread executing the machine
      - two threads for mavlink api*/

    //intialize Node basic
    node = std::unique_ptr<Node<data_type>>(new Node<data_type>(SamplingTime, ControlTime));
    node = std::unique_ptr<PathPlanner<data_type>>(new PathPlanner<data_type>(SamplingTime, ControlTime));
    //ChangeType(NodeType::Leader);
    //Initialize client thread for receiving and sending data to server

    
    #ifdef HITL
    /* Initilize Mavlink Control */
    InitializeMavlinkConnection(argc,argv);

    // --------------------------------------------------------------------------
    //  Start Mavlink Threads -  PORT and THREAD STARTUP
    // --------------------------------------------------------------------------

        /*
	 * Instantiate a serial port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * serial port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock.
	 *
	 */
        Serial_Port serial_port(uart_name, baudrate);

    /*
	 * Instantiate an autopilot interface object
	 *
	 * This starts two threads for read and write over MAVlink. The read thread
	 * listens for any MAVlink message and pushes it to the current_messages
	 * attribute.  The write thread at the moment only streams a position target
	 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
	 * is changed by using the method update_setpoint().  Sending these messages
	 * are only half the requirement to get response from the autopilot, a signal
	 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
	 * method.  Signal the exit of this mode with disable_offboard_control().  It's
	 * important that one way or another this program signals offboard mode exit,
	 * otherwise the vehicle will go into failsafe.
	 *
	 */
    Autopilot_Interface autopilot_interface(&serial_port);

    /*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
    serial_port_quit = &serial_port;
    autopilot_interface_quit = &autopilot_interface;
    

    /*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
    serial_port.start();
    autopilot_interface.start();

    //store systemID
    formationData.SetCurrentDroneID(autopilot_interface.system_id);
    #else

    std::ifstream file;

    file.open("droneid.config");
    if (file)
    {
        unsigned int currentDroneId = 0;
        file >> currentDroneId;
        file.close();
        formationData.SetCurrentDroneID(currentDroneId);
    }
    else
    {
        printf("no current drone id fixed\n");
        exit(0);
    }
    

    

    #endif

    //load server details
    file.open("serverdetails.config");
    if (file)
    {
        std::string line;
        file >> line;
        file.close();
        std::vector<std::string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(" :="));
        if (vec[0].compare("ipaddress")==0)
        {
            printf("configuring ip address\n");
            std::strcpy(ServerDetails::SERVER,vec[1].c_str());
        }

            
    }
    else
    {
        printf("no server details configured");
            exit(0);
    }

    //Initialize Timer
    InitializeTimer();

    

    signal(SIGINT, quit_handler);

    srand(time(0)); // Initialize random number generator.

    //Initialize DataSync Client
    Client<data_type> dataSyncClient;
    dataSyncClient.Start();
    dataSyncClient_Quit = &dataSyncClient;



    while (!autoPilotData.ShouldExit())
    {
        
            //need to enable offboard control
        if (node->execute)
        {            
            
            #ifdef HITL
            UpdateCurrentDroneData(autopilot_interface.current_messages);
            if (autoPilotData.RunControlAlgorthm())
                node->Execute(formationData, autopilot_interface);
            #else
            node->execute = false;
            UpdateCurrentDroneData(node);
            if(autoPilotData.RunControlAlgorthm())
                node->Execute(formationData);
            #endif
        }
        else
        {
            usleep(50);
        }
      
        if(node->nodeType != autoPilotData.GetNodeType())
        {
            ChangeType(autoPilotData.GetNodeType());
        }

        if(formationData.IsNewPathPlannerDataAvailable())
        {
            
            node->InitializeType();
        }
        
    }

    signal(SIGALRM, SIG_IGN);
    dataSyncClient.Stop();

    printf("\n");
    printf("TERMINATING AT SERVER PROGRAM REQUEST\n");
    printf("\n");

    #ifdef HITL
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------

    /*
	 * Now that we are done we can stop the threads and close the port
	 */
    autopilot_interface.stop();
    serial_port.stop();

    #endif

    // --------------------------------------------------------------------------
    //   DONE
    // --------------------------------------------------------------------------

    // woot!
    return 0;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function intializes mavlink related things

void InitializeMavlinkConnection(int argc, char **argv)
{

    //default values
    uart_name = (char *)"/dev/ttyUSB0";
    baudrate = 57600;

    parse_commandline(argc, argv, uart_name, baudrate);
}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

    // string for command line usage
    const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

    // Read input arguments
    for (int i = 1; i < argc; i++)
    { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
        {
            printf("%s\n", commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0)
        {
            if (argc > i + 1)
            {
                uart_name = argv[i + 1];
            }
            else
            {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0)
        {
            if (argc > i + 1)
            {
                baudrate = atoi(argv[i + 1]);
            }
            else
            {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }
    }
    // end: for each input argument

    // Done!
    return;
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

    #ifdef HITL
    // autopilot interface
    try
    {
        autopilot_interface_quit->handle_quit(sig);
    }
    catch (int error)
    {
    }

    // serial port
    try
    {
        serial_port_quit->handle_quit(sig);
    }
    catch (int error)
    {
    }
    #endif
    //stop DataSync Client
    dataSyncClient_Quit->Stop();

    signal(SIGALRM, SIG_IGN);
    // end program here
    exit(0);
}


int count2 = 0;
void timer_handler(int signum)
{
    //printf("interupted %d times\n", ++count2);
    node->HandleTimerInterrupt();
}

void 
InitializeTimer()
{
    /* Install timer_handler as the signal handler for SIGVTALRM. */
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = &timer_handler;
    sigaction(SIGALRM, &sa, NULL);

    /* Configure the timer to expire after seconds msec... */
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = SamplingTimeus;
    /* ... and every 250 msec after that. */
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = SamplingTimeus;
    /* Start a virtual timer. It counts down whenever this process is
	executing. */
    setitimer(ITIMER_REAL, &timer, NULL);
}

#ifdef HITL
void UpdateCurrentDroneData(const Mavlink_Messages messages)
{
    aDroneData<data_type> currentDroneData;

    //position - WRONG
    currentDroneData.droneStates[0]= messages.local_position_ned.x;
    currentDroneData.droneStates[1] = messages.local_position_ned.y;
    currentDroneData.droneStates[2] = messages.local_position_ned.z;
    printf("    pos  (NED):  %f %f %f (m)\n", currentDroneData.droneStates[0], currentDroneData.droneStates[1], currentDroneData.droneStates[2]);

    //attitude
    currentDroneData.droneStates[3] = messages.attitude.roll;
    currentDroneData.droneStates[4] = messages.attitude.pitch;
    currentDroneData.droneStates[5] = messages.attitude.yaw;

    //velocity in body frame
    currentDroneData.droneStates[6] = messages.local_position_ned.vx;
    currentDroneData.droneStates[7] = messages.local_position_ned.vy;
    currentDroneData.droneStates[8] = messages.local_position_ned.vz;

    //angular velocity
    currentDroneData.droneStates[9] = messages.attitude.rollspeed;
    currentDroneData.droneStates[10] = messages.attitude.pitchspeed;
    currentDroneData.droneStates[11] = messages.attitude.yawspeed;
    formationData.UpdateDroneData(currentDroneData);
}
#else
void UpdateCurrentDroneData(std::unique_ptr<Node<data_type>>& node)
{
    DroneData<data_type> currentDroneData;
    currentDroneData.droneID = formationData.GetCurrentDroneID();
    node->FillPlantData(currentDroneData);
    
    // for(auto i = 0;i<12;i++)
    // {
    //     currentDroneData.droneStates[i] = currentDroneStates[i];
    // }
    //currentDroneData.droneStates[10] =(rand()%10)*2 + -10;
    
    //std::cout << currentDroneData.droneStates[12]<<std::endl;
    formationData.UpdateDroneData(currentDroneData);
}

#endif

