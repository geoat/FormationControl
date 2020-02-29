#include <iostream>


#include <type_traits>
#include <memory>

#include <Eigen/Dense>
#include "DroneSim/dronemodel.hpp"
#include "Environment/windmodel.hpp"

#include "pathmanager.hpp"

#include "dronewithidc.hpp"

#include <fstream>

    typedef double data_type;
    typedef Eigen::Matrix<data_type, 3, 1>  vector_type;
    typedef Eigen::Matrix<data_type, 3, 3> matrix_type;
    typedef Eigen::Matrix<data_type, 6, 1> q_type;

int main()
{
  //sinks
  std::ofstream pathplannerStream;
  std::ofstream virtualDroneStream;
  pathplannerStream.open("pathplanner_output.Csv");
  virtualDroneStream.open("virtualdrone_output.Csv");


    data_type Ts =       0.0025;
    data_type Ts2 = Ts*100;
    //defining path
    auto point1 = Point<data_type>(0,0);
    auto point2 = Point<data_type>(500,500);
    auto point3 = Point<data_type>(500,0);
    auto point4 = Point<data_type>(0,500);
    auto point5 = Point<data_type>(0,0);
    //Way<data_type> way(0.0025,30,point1,point2,point3,point4,point5);
    //std::cout<<"Ts:"<<Ts<<std::endl;
    PathManager<data_type> pathmanager(Ts2);
    pathmanager.SetPathAsWay(30,point1,point2,point3,point4,point5,point2,point3);

    Point<data_type> location(1,1);
    Point<data_type> location2(1,1);
    
    data_type Mass =1;
    matrix_type I;
    I<<0.02,0,-0.01,
      0,0.026,0,
      -0.01,0,0.053;


  //setting intial conditions for states
  //xyz initial
  //xyz = [0,500,-50]';

    vector_type Xe_init;
    Xe_init<<0,500,-50;

    vector_type Xe_init2;
    Xe_init2<<0,-500,-50;

  //uvw initial
  //uvw = [15,0,0]';
    vector_type Vb_init;
    Vb_init<<15,0,0;
  //euler initial
  //euler = [2;3;1];
    vector_type Euler_init;
    Euler_init<<0.1,0.1,1;

    vector_type Euler_init2;
    Euler_init2<<0.1,0.1,-1;

    
  //pqr initial
  //pqr = [1;1;2];
    vector_type pqr_init;
    pqr_init<<1,1,2;

    vector_type pqr_init2;
    pqr_init2<<-1,-1,-2;


    int count = (Ts2/Ts);
    //Initializing Plant Parameters
    Mass =1.0;
    Drone<data_type,true,true> drone(Ts,Mass,Xe_init,Vb_init,Euler_init,pqr_init);
    data_type Mass2 =1.0;
    DroneWithIDC<data_type> droneWithIDC(Ts,Mass2,I,Xe_init2,Vb_init,Euler_init2,pqr_init2);

    data_type HT = 50;
    data_type VT = 15;
    data_type WindVector[3] = {0,0,0};

    WindModel<data_type,true> windModel;
    data_type chi_c = 0;
    for(data_type t = 0; t < 200; t = t+Ts)
    {
        location.x=drone.y[0];
        location.y=drone.y[1];
        location2.x=droneWithIDC.y[0];
        location2.y=droneWithIDC.y[1];

        
        //std::cout<<location.x-location.y<<std::endl;
        auto windData= windModel.GetWindData(t);
        //std::cout<<",wind="<<windData.TotalWind.Value[0]<<","<<windData.TotalWind.Value[1]<<",";

        DerivedPlantData<data_type> derivedData(drone.y,windData.TotalWind.Value);
        
        if(count==(Ts2/Ts))
        {
            count = 0;
            chi_c = pathmanager.path->GetChi_c(derivedData,location,&(drone.y[3]));
        }
        count++;       
        
    
        std::cout<<location.x<<","<<location.y<<std::endl;//<<","<<chi_c*180/M_PI<<std::endl;
        pathplannerStream<<location.x<<","<<location.y<<std::endl;
        std::cout<<location2.x<<","<<location2.y<<std::endl;//<<","<<chi_c*180/M_PI<<std::endl;
        virtualDroneStream<<location2.x<<","<<location2.y<<std::endl;
        q_type rTrajectory = ReferenceModel<data_type>::GetReferenceModel().CalculateReferenceInput(drone.y,derivedData,drone.acceleration,drone.wbdot);
        
        //std::cout<<chi_c<<std::endl;
        drone.wind_vector[0] = windData.TotalWind.x;
        drone.wind_vector[1] = windData.TotalWind.y;
        drone.wind_vector[2] = windData.TotalWind.z;
        //std::cout<<"Wind:"<<drone.wind_vector[0]<<","<<drone.wind_vector[1]<<","<<drone.wind_vector[2]<<","<<std::endl;
        drone.DoNextStep(chi_c*180/M_PI,VT,HT);
        //std::cout<<drone.acceleration[0]<<","<<drone.acceleration[1]<<","<<drone.acceleration[2]<<std::endl;
        //std::cout<<drone.wbdot[0]<<","<<drone.wbdot[1]<<","<<drone.wbdot[2]<<std::endl;

        //std::cout<<drone.wbdot<<std::endl;

        droneWithIDC.DoNextStep(rTrajectory,ReferenceModel<data_type>::GetReferenceModel());
    }

pathplannerStream.close();
virtualDroneStream.close();   



}