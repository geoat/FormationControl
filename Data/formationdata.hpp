#ifndef FORMATIONDATA_HPP
#define FORMATIONDATA_HPP

#include "formationconfig.hpp"
#include "dronedata.hpp"


#include <pthread.h>

#include <iostream>

#include <vector>
#include <algorithm>
#include <map>

#include "../Types/formationtypes.hpp"
//for communication graph
#include "communicationgraph.hpp"
#include "pathplannerdata.hpp"
#include "../Timing/timing.hpp"

template<typename data_type = double>
class FormationData
{
    private:
    pthread_mutex_t lock;
    pthread_mutex_t sourceDroneslock;
    pthread_mutex_t pathPlannerDataLock;

    _droneID_type currentDroneID;
    std::vector<_droneID_type> sourceDrones;
    //DroneData<data_type> AllDronesData[ConstellationSize+1];
    std::map<_droneID_type,DroneData<data_type>> droneDataMap;

    
    PathPlannerData<data_type> inputPathPlannerData;
    bool newPathPlannerDataAvailable = false;
    
    FormationData()
    {
        sourceDrones.reserve(2);
        InitializeDafaults();
    }

    inline int InitializeDafaults()
    {
        if (pthread_mutex_init(&lock, NULL) != 0)
        {
            printf("\n mutex init failed\n");
            exit(0);
        }

        if (pthread_mutex_init(&sourceDroneslock, NULL) != 0)
        {
            printf("\n sourceDroneslock mutex init failed\n");
            exit(0);
        }

        if (pthread_mutex_init(&pathPlannerDataLock, NULL) != 0)
        {
            printf("\n pathPlannerDataLock mutex init failed\n");
            exit(0);
        }
    }

    public:
        FormationData(FormationData<data_type> const &) = delete;
        void operator=(FormationData<data_type> const &) = delete;

        static FormationData<data_type> &GetFormationData()
        {
            static FormationData<data_type> instance; // Guaranteed to be destroyed.
                                                       // Instantiated on first use.
            return instance;
        }

        ~FormationData()
        {
            pthread_mutex_destroy(&lock);
            pthread_mutex_destroy(&sourceDroneslock);
            pthread_mutex_destroy(&pathPlannerDataLock);
        }

        /** GetCurrentDroneID
        Gets current Drones ID in the formation Data reported through SetCurrentDroneID
        ***MUST BE CALLED BEFORE USING ANY FORMATION CONTROL FEATURES****

        @return unsigned int current drone id.
        */
        _droneID_type GetCurrentDroneID()
        {
            return currentDroneID;        
        }

    /** SetCurrentDroneID
        Sets Current Drones ID in the formation Data 
        ***MUST BE CALLED BEFORE USING ANY FORMATION CONTROL FEATURES****

        @param droneID id of the current drone.
        @return NIL
        */
    void SetCurrentDroneID(const _droneID_type droneID)
    {
        currentDroneID = droneID;
        //AllDronesData[currentDroneID].droneID = currentDroneID;
        DroneData<data_type> droneData;
        droneData.droneID = droneID;
        droneDataMap.emplace(droneID, droneData);
    }

    

    /** GetSourceDrones
        Gets source drones for the current drones algorithm.

        @param result std::vector<_droneID_type>& vector of source drones.
        */
    void GetSourceDrones(std::vector<_droneID_type>& result)
    {

        pthread_mutex_lock(&sourceDroneslock);
            result = sourceDrones;
            //printf("sourceDrones.size():%u", sourceDrones.size());
        pthread_mutex_unlock(&sourceDroneslock);
    }

    /** SetSourceDrones
        Stores targetted drones for a specific drones data. Also reserves data for storing the information of such drones in data map

        @param droneIDs unsigned char* Array of drone ids.
        @param numberOfSourceDrones char Number of drones.
        */
    void SetSourceDrones(unsigned char* droneIDs,char numberOfSourceDrones)
    {

        pthread_mutex_lock(&sourceDroneslock);
        sourceDrones.clear();
        
        sourceDrones.reserve(numberOfSourceDrones);


        for (size_t i = 0; i < numberOfSourceDrones; i++)
        {
            sourceDrones.push_back((_droneID_type) droneIDs[i]);

            auto search = droneDataMap.find(droneIDs[i]);
            if (search == droneDataMap.end()){
                DroneData<data_type> droneData;
                droneData.droneID = droneIDs[i];
                droneDataMap.emplace(droneIDs[i], droneData);
            }
        }
        std::cout << sourceDrones.size() << std::endl;


        
        pthread_mutex_unlock(&sourceDroneslock);

    }

    
    // FormationData(unsigned int &currentDroneID) : currentDroneID(currentDroneID)
    // {
    //     InitializeDafaults();
    // }



    inline void 
    GetDroneData(const unsigned int& droneID, DroneData<data_type> &data)
    {
        pthread_mutex_lock(&lock);
        data = droneDataMap[droneID];
        //data = AllDronesData[droneID];
        pthread_mutex_unlock(&lock);
    }

    inline void GetCurrentDroneData(DroneData<data_type> &data)
    {
        pthread_mutex_lock(&lock);
        data = droneDataMap[currentDroneID];
        //data = AllDronesData[currentDroneID];
        pthread_mutex_unlock(&lock);
    }

    inline void UpdateDroneData(DroneData<data_type> &data)
    {
        pthread_mutex_lock(&lock);
        
        data.lastReportedTime = GetCurrentTime_ms();
        //AllDronesData[data.droneID] = data;
        droneDataMap[data.droneID] = data;
        //std::cout << "Data"<<AllDronesData[data.droneID].droneStates[10] << std::endl;

        pthread_mutex_unlock(&lock);
    }

    void ReportNewPathPlannerData(const PathPlannerData<data_type>& newPathPlannerData)
    {
        pthread_mutex_lock(&pathPlannerDataLock);
        this->inputPathPlannerData = newPathPlannerData;
        this->newPathPlannerDataAvailable = true;
        pthread_mutex_unlock(&pathPlannerDataLock);
    }

    bool IsNewPathPlannerDataAvailable()
    {
        bool result = false;
        pthread_mutex_lock(&pathPlannerDataLock);
        result = newPathPlannerDataAvailable;
        pthread_mutex_unlock(&pathPlannerDataLock);

        return result;
    }

    void ClearNewPathPlannerDataAvailableFlag()
    {
        pthread_mutex_lock(&pathPlannerDataLock);
        newPathPlannerDataAvailable = false;
        pthread_mutex_unlock(&pathPlannerDataLock);
    }

        PathPlannerData<data_type>
        GetPathPlannerData()
    {
        PathPlannerData<data_type> pathPlannerData;
        pthread_mutex_lock(&pathPlannerDataLock);
        pathPlannerData = this->inputPathPlannerData;
        pthread_mutex_unlock(&pathPlannerDataLock); 

        return pathPlannerData;       
    }

    
};




#endif
