
#pragma once

#include "../Data/followerdata.hpp"
#include "../Data/controloutput.hpp"
#include "../Data/formationdata.hpp"
#include "../Data/dronedata.hpp"
#include "../Data/communicationgraph.hpp"
#include "../Types/formationtypes.hpp"
#include "../Timing/timing.hpp"
#include "followerestimator.hpp"
//For using Eigen Library for matrix calculations
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <iostream>
template<typename data_type>
class FollowerController
{
    std::vector<_droneID_type> sourceDrones;
    public:
        typedef Eigen::Matrix<data_type, 6, 1> eigen_vector6_type;
        typedef Eigen::Matrix<data_type, 3, 1> eigen_vector3_type;

        FormationData<data_type> &formationData;
        FollowerController(
            data_type &integrationTime) : formationData(FormationData<data_type>::GetFormationData()),
                                          followerEstimator(followerData, integrationTime)
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

            DroneData<data_type> sourceDroneData;
            formationData.GetDroneData(sourceDroneID,sourceDroneData);

            if (!sourceDroneData.IsTimeValid(CLIENT_MSG_CONNECTION_LOST_TIME_PERIOD_MS))
            {

                return false;
            }
            //printf("calculation reached\n");
            //map state variables
            data_type *referenceDroneStates_addr = sourceDroneData.droneStates.data();
            data_type Xbi_arr[3];
            std::copy(referenceDroneStates_addr, referenceDroneStates_addr + 3, Xbi_arr);
            RotateFromEarthToBody(Xbi_arr, referenceDroneStates_addr + 6);

            Eigen::Map<eigen_vector3_type> Xbi(Xbi_arr);
            Eigen::Map<eigen_vector3_type> Vbi(referenceDroneStates_addr + 3);
            Eigen::Map<eigen_vector3_type> euleri(referenceDroneStates_addr + 6);
            Eigen::Map<eigen_vector3_type> Wbi(referenceDroneStates_addr + 9);
            Eigen::Map<eigen_vector6_type> taui(sourceDroneData.controlInput.data());

            data_type *currentDroneStates_addr = currentDroneData.droneStates.data();
            data_type Xbn_arr[3];
            std::copy(currentDroneStates_addr, currentDroneStates_addr + 3, Xbn_arr);
            RotateFromEarthToBody(Xbn_arr, currentDroneStates_addr + 6);

            Eigen::Map<eigen_vector3_type> Xbn(Xbn_arr);
            Eigen::Map<eigen_vector3_type> Vbn(currentDroneStates_addr + 3);
            Eigen::Map<eigen_vector3_type> eulern(currentDroneStates_addr + 6);
            Eigen::Map<eigen_vector3_type> Wbn(currentDroneStates_addr + 9);

            //calculate error eni_1
            eigen_vector6_type eni_1, eni_2;
            eni_1 << Xbn - Xbi, eulern - euleri;

            //calculate error eni_2
            eni_2 << Vbn - Vbi, Wbn - Wbi;

            //calculate al(-Kpq-Kvq_dot+r)
            ReferenceModel<data_type>::GetReferenceModel().Calculate_aFollower(followerData.al.data(), eni_1.data(), eni_2.data());

            //calculate Xi matrices
            followerData.Setxi(eulern.data(),Wbn.data(),euleri.data(),Wbi.data());



            //printf("calculation reached\n");
            //calculate estimates
            followerEstimator.EstimateTheta(
            Xbi,
            Vbi,
            euleri,
            Wbi,
            Xbn,
            Vbn,
            eulern,
            Wbn,
            taui
            );
            
            eigen_vector6_type tauIn;
            
            //calculate q,qdot of Follower
            eigen_vector6_type qn_dot, qi_dot;
            qn_dot << Vbn, Wbn;
            qi_dot << Vbi, Wbi;
            //Eigen::Map<eigen_vector6_type> al(followerData.al.data());
            tauIn = -followerEstimator.Theta_DnT * followerData.xi_Dn * followerData.al +
                    followerEstimator.Theta_CnT * followerData.xi_Cn * qn_dot +
                    followerEstimator.Theta_DnDiT * followerData.xi_DnDi * taui -
                    followerEstimator.Theta_DnDiCiT * followerData.xi_DnDiCi * qi_dot + 
                    followerEstimator.Theta_gnT * followerData.xi_gn;

            data_type* tauIn_arr = tauIn.data();
            std::copy(tauIn_arr,tauIn_arr+3,controlOutput.F);
            std::copy(tauIn_arr+3,tauIn_arr+6,controlOutput.M);

            
        }
        else
        {
            
            return false;
        }
        return true;


    }

    private:
    FollowerData<data_type> followerData;
    FollowerEstimator<data_type> followerEstimator;


        
};