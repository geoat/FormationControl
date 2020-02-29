
#pragma once

#include "../Data/leaderdata.hpp"
#include "../Data/controloutput.hpp"
#include "../Data/formationdata.hpp"
#include "../Data/dronedata.hpp"
#include "../Data/communicationgraph.hpp"
#include "../Types/formationtypes.hpp"
#include "../Timing/timing.hpp"
#include "leaderestimator.hpp"
//For using Eigen Library for matrix calculations
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <iostream>
template<typename data_type = double>
class LeaderController
{
    std::vector<_droneID_type> sourceDrones;
    public:
        typedef Eigen::Matrix<data_type, 6, 1> eigen_vector6_type;
        typedef Eigen::Matrix<data_type, 3, 1> eigen_vector3_type;

        FormationData<data_type> &formationData;
        LeaderController(
            data_type &integrationTime) : formationData(FormationData<data_type>::GetFormationData()),
                                          leaderEstimator(leaderData, integrationTime)
        {

    }

    bool CalculateFandM(ControlOutput<data_type>& controlOutput)
    {
        //get source ones
        formationData.GetSourceDrones(sourceDrones);
        //printf("reached\n");
        if(!sourceDrones.empty()){
            _droneID_type sourceDroneID = sourceDrones[0];
            
            //get formation data
            DroneData<data_type> currentDroneData;
            formationData.GetCurrentDroneData(currentDroneData);
            
            DroneData<data_type> referenceDroneData;
            formationData.GetDroneData(sourceDroneID,referenceDroneData);
            //printf("Drone%d,x=%f,y=%f,Time=%u\n", referenceDroneData.droneID, referenceDroneData.droneStates[0], referenceDroneData.droneStates[1], referenceDroneData.lastReportedTime);
             if (!referenceDroneData.IsTimeValid(CLIENT_MSG_CONNECTION_LOST_TIME_PERIOD_MS))
            {

                return false;
            }
            //printf("calculation reached\n");
            //map state variables
            data_type *referenceDroneStates_addr = referenceDroneData.droneStates.data();
            data_type Xbm_arr[3];
            std::copy(referenceDroneStates_addr, referenceDroneStates_addr + 3, Xbm_arr);
            RotateFromEarthToBody(Xbm_arr, referenceDroneStates_addr + 6);

            Eigen::Map<eigen_vector3_type> Xbm(Xbm_arr);
            Eigen::Map<eigen_vector3_type> Vbm(referenceDroneStates_addr + 3);
            Eigen::Map<eigen_vector3_type> eulerm(referenceDroneStates_addr + 6);
            Eigen::Map<eigen_vector3_type> Wbm(referenceDroneStates_addr + 9);

            data_type *currentDroneStates_addr = currentDroneData.droneStates.data();
            data_type Xbl_arr[3];
            std::copy(currentDroneStates_addr, currentDroneStates_addr + 3, Xbl_arr);
            RotateFromEarthToBody(Xbl_arr, currentDroneStates_addr + 6);

            Eigen::Map<eigen_vector3_type> Xbl(Xbl_arr);
            Eigen::Map<eigen_vector3_type> Vbl(currentDroneStates_addr + 3);
            Eigen::Map<eigen_vector3_type> eulerl(currentDroneStates_addr + 6);
            Eigen::Map<eigen_vector3_type> Wbl(currentDroneStates_addr + 9);

            
            //calculate al(-Kpq-Kvq_dot+r)
            ReferenceModel<data_type>::GetReferenceModel().Calculate_aLeader(leaderData.al.data(), referenceDroneData.controlInput.data(), Xbl.data(), eulerl.data(), Vbl.data(), Wbl.data());

            //calculate Xi matrices
            leaderData.Setxi(eulerl.data(),Wbl.data());
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
            Wbl
            );
            
            eigen_vector6_type Tau_l;
            
            //calculate q,qdot of Leader
            eigen_vector6_type ql_dot;
            ql_dot << Vbl, Wbl;
            //Eigen::Map<eigen_vector6_type> al(leaderData.al.data());
            Tau_l = leaderEstimator.Theta_DT * leaderData.xi_Dn * leaderData.al +
                    leaderEstimator.Theta_CT * leaderData.xi_Cn * ql_dot +
                    leaderEstimator.Theta_gT * leaderData.xi_gn;

            data_type* Tau_l_arr = Tau_l.data();
            std::copy(Tau_l_arr,Tau_l_arr+3,controlOutput.F);
            std::copy(Tau_l_arr+3,Tau_l_arr+6,controlOutput.M);

            
        }
        else
        {
            
            return false;
        }
        return true;


    }

    private:
    LeaderData<data_type> leaderData;
    LeaderEstimator<data_type> leaderEstimator;


        
};