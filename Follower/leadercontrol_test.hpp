#pragma once


#include <iostream>
#include <cstdlib>
#include <iomanip>
#include <cmath>
#include <ctime>
#include <fstream>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>

#include "../Data/leaderdata.hpp"
#include "../Data/controloutput.hpp"
#include "../Data/formationdata.hpp"
#include "../Data/dronedata.hpp"
#include "../DroneSim/derivedplantdata.hpp"
#include "../Data/communicationgraph.hpp"
#include "../Types/formationtypes.hpp"
#include "../Timing/timing.hpp"
#include "leaderestimator.hpp"
//For using Eigen Library for matrix calculations
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <iostream>


void testLeaderEstimator()
{

    typedef double data_type;
    typedef std::vector<data_type> state_type;

    typedef Eigen::Matrix<data_type, 6, 1> eigen_vector6_type;
    typedef Eigen::Matrix<data_type, 3, 1> eigen_vector3_type;

    std::ifstream file;

    using namespace std;

    //*****************************leader control test - estimation laws***********************************
    std::cout << "leader control test - estimation laws" << std::endl;
    file.open("Leader/leaderControlInput.Csv");
    std::string line = "";
    clock_t begin = clock();
    state_type referenceStates{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    state_type referenceAcc{0, 0, 0};
    state_type referenceAngAcc{0, 0, 0};

    // Iterate through each line and split the content using delimeter
    std::getline(file, line);
    
    std::vector<std::string> vec;
    boost::algorithm::split(vec, line, boost::is_any_of(","));
    
    LeaderData<data_type> leaderData;
    double sampleTime = 0.0025;
    LeaderEstimator<data_type> leaderEstimator(leaderData, sampleTime);
    
    for (size_t i = 0; i < 12; i++)
    {
        referenceStates[i] = atof(vec[i].c_str());
    }
    
    for (size_t i = 0; i < 3; i++)
    {
        referenceAcc[i] = atof(vec[12+i].c_str());
        //std::cout<<drone3.y[i]<<std::endl;
    }

    for (size_t i = 0; i < 3; i++)
    {
        referenceAngAcc[i] = atof(vec[15+i].c_str());
        //std::cout<<drone3.y[i]<<std::endl;
    }
    
    state_type droneStates{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    std::getline(file, line);
    boost::algorithm::split(vec, line, boost::is_any_of(","));
    std::cout << "leader control test - estimation laws - reached" << std::endl;
    sleep(1);
    for (size_t i = 0; i < 12; i++)
    {
        droneStates[i] = atof(vec[i].c_str());
    }
    
    //map state variables
    data_type *referenceDroneStates_addr = referenceStates.data();
    data_type Xbm_arr[3];
    std::copy(referenceDroneStates_addr, referenceDroneStates_addr + 3, Xbm_arr);
    RotateFromEarthToBody(Xbm_arr, referenceDroneStates_addr + 6);

    Eigen::Map<eigen_vector3_type> Xbm(Xbm_arr);
    Eigen::Map<eigen_vector3_type> Vbm(referenceDroneStates_addr + 3);
    Eigen::Map<eigen_vector3_type> eulerm(referenceDroneStates_addr + 6);
    Eigen::Map<eigen_vector3_type> Wbm(referenceDroneStates_addr + 9);

    data_type *currentDroneStates_addr = droneStates.data();
    data_type Xbl_arr[3];
    std::copy(currentDroneStates_addr, currentDroneStates_addr + 3, Xbl_arr);
    RotateFromEarthToBody(Xbl_arr, currentDroneStates_addr + 6);

    Eigen::Map<eigen_vector3_type> Xbl(Xbl_arr);
    Eigen::Map<eigen_vector3_type> Vbl(currentDroneStates_addr + 3);
    Eigen::Map<eigen_vector3_type> eulerl(currentDroneStates_addr + 6);
    Eigen::Map<eigen_vector3_type> Wbl(currentDroneStates_addr + 9);

    DerivedPlantData<data_type> derivedData(referenceStates);
    Eigen::Matrix<data_type, 6, 1> rTrajectory = ReferenceModel<data_type>::GetReferenceModel().CalculateReferenceInput(referenceStates, derivedData, referenceAcc, referenceAngAcc);

    //calculate al(-Kpq-Kvq_dot+r)
    ReferenceModel<data_type>::GetReferenceModel().Calculate_aLeader(leaderData.al.data(), rTrajectory.data(), Xbl.data(), eulerl.data(), Vbl.data(), Wbl.data());
        
    //calculate Xi matrices
    leaderData.Setxi(eulerl.data(), Wbl.data());
    //std::cout << "leaderData.xi_Cn\n" << leaderData.xi_Cn<< std::endl;
    //std::cout << "leaderData.xi_gn\n" << leaderData.xi_gn<< std::endl;
    //printf("calculation reached\n");
    //calculate estimates
    leaderEstimator.EstimateTheta(
        Xbm,
        Vbm,
        eulerm,
        Wbm,
        Xbl,
        Vbl,
        eulerl,
        Wbl);

    clock_t end = clock();
    file.close();
}