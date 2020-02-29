#include <sys/time.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>

//for control
#include "DroneSim/derivedplantdata.hpp"
#include "pathmanager.hpp"
#include "Environment/windmodel.hpp"


bool execute = false;
  int count2 = 0;

typedef double data_type;
typedef std::vector< data_type > drone_states;
drone_states droneStates = drone_states(12);

//defining path
auto point1 = Point<data_type>(0,0);
auto point2 = Point<data_type>(500,500);
auto point3 = Point<data_type>(500,0);
auto point4 = Point<data_type>(0,500);
auto point5 = Point<data_type>(0,0);
Point<data_type> location(1,1);
data_type Ts = 0.0025;
PathManager<data_type> pathmanager(Ts);


  
void timer_handler1 (int signum)
{
 printf ("1interupted %d times\n", ++count2);
 execute = true;
}

int calculationLoopInterval_us = Ts*1000000;
int controlCommandDespatch_At = 40;
WindModel<data_type, false> windModel;

int main ()
{
 struct sigaction sa;
 struct itimerval timer;

 /* Install timer_handler as the signal handler for SIGVTALRM. */
 memset (&sa, 0, sizeof (sa));
 sa.sa_handler = &timer_handler1;
 sigaction (SIGALRM, &sa, NULL);

 /* Configure the timer to expire after 250 msec... */
 timer.it_value.tv_sec = 0;
 timer.it_value.tv_usec = calculationLoopInterval_us;
 /* ... and every 250 msec after that. */
 timer.it_interval.tv_sec = 0;
 timer.it_interval.tv_usec = calculationLoopInterval_us;
 /* Start a virtual timer. It counts down whenever this process is
   executing. */
 setitimer (ITIMER_REAL, &timer, NULL);
  int count = 0;
  int controlLoopCount = 0;
  /* Do busy work. */

  pathmanager.SetPathAsWay(30, point1, point2, point3, point4, point5, point2, point3);

  while (count < 1000)
  {
    
    if(execute)
          {

            execute = false;
          controlLoopCount++; 
          count++;
                
          // copy current messages
            //Mavlink_Messages messages = api.current_messages;
            //SetDroneStateVector(messages);
          
          // DerivedPlantData<data_type> derivedDroneData(droneStates, windModel.ConstantComponent.Value);
          // printf("    pos  (NED):  %f %f %f (m)\n", droneStates[0], droneStates[1], droneStates[2]);
          // printf("    att  (NED):  %f %f %f (m)\n", droneStates[3], droneStates[4], droneStates[5]);
          // printf("    vel  (NED):  %f %f %f (m)\n", droneStates[6], droneStates[7], droneStates[8]);
          // printf("    w  (NED):  %f %f %f (m)\n", droneStates[9], droneStates[10], droneStates[11]);
          // data_type chi_c = pathmanager.path->GetChi_c(derivedDroneData, location, &(droneStates[6]));
          // std::cout << "chi_c:" << chi_c << std::endl;
          // std::cout << "wind_Ws:" << wind_Ws << std::endl;
          // std::cout << "wind_phiw:" << wind_phiw << std::endl;
          if(controlLoopCount==controlCommandDespatch_At)
          {
            controlLoopCount = 0;

            //send control command
            //set_yaw( chi_c , // [rad]
            //sp     );

            // SEND THE COMMAND
            //api.update_setpoint(sp);
            

          }

        }
        else
        {
          usleep(50); 
        }

 }

 signal(SIGALRM, SIG_IGN);




}