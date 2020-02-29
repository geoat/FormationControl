#pragma once
#include "Types/formationtypes.hpp"

#include <vector>
#include <map>
#include "../Timing/timing.hpp"

#include "configuration.hpp"

#include "Eigen/Sparse"

#define DATA_DISTRIBUTION_INTEVRAL_MS 100

class CommunicationGraph
{
    private:
    std::map<_droneID_type, std::vector<_droneID_type>> dataLinks;
    std::map<_droneID_type, _time_type> nextSendTime;
    pthread_mutex_t communicationGraphlock;

    std::vector < _droneID_type> keys;

    int InitializeDafaults()
    {
        if (pthread_mutex_init(&communicationGraphlock, NULL) != 0)
        {
            printf("\n mutex init failed\n");
            return 1;
        }
    }

    CommunicationGraph()
    {
        InitializeDafaults();
    }

    /** Constructor
        Constructs object of class CommunicationGraph

        @param droneIDsInFormation _droneID_type[] IDs of drones participting in formation.
        @param numberOFDronesInFomration char total number of drones in the formation.
        @param AdjacencyMatrix bool* pointer to first element (i=0,j=0) of Adjacency Matrix of communication graph.
        @return NIL
        */
    template <size_t rows, size_t cols>
    CommunicationGraph(_droneID_type droneIDsInFormation[], size_t numberOFDronesInFomration, bool (&AdjacencyMatrix)[rows][cols]) : CommunicationGraph()
    {
        SetGraph(droneIDsInFormation, numberOFDronesInFomration, AdjacencyMatrix);
    }

public:
    CommunicationGraph(CommunicationGraph const &) = delete;
    void operator=(CommunicationGraph const &) = delete;

    static CommunicationGraph &GetCommunicationGraph()
    {
        static CommunicationGraph instance; // Guaranteed to be destroyed.
                                                  // Instantiated on first use.
        return instance;
    }

    ~CommunicationGraph()
    {
        pthread_mutex_destroy(&communicationGraphlock);
    }

    void print(std::vector<_droneID_type> const &input)
    {
        for (int i = 0; i < input.size(); i++)
        {
            printf("%d,", input.at(i));
        }
        printf("\n,");
    }


    /** SetGraph
        Sets CommunicationGraph

        @param droneIDsInFormation _droneID_type[] IDs of drones participting in formation.
        @param numberOFDronesInFomration char total number of drones in the formation.
        @param AdjacencyMatrix bool* pointer to first element (i=0,j=0) of Adjacency Matrix of communication graph.
        @return NIL
        */
    template <size_t rows, size_t cols>
    void SetGraph(_droneID_type droneIDsInFormation[], size_t numberOFDronesInFomration, bool (&AdjacencyMatrix)[rows][cols])
    {
        pthread_mutex_lock(&communicationGraphlock);
        dataLinks.clear();
        nextSendTime.clear();
        keys.clear();
        for (size_t j = 0; j < numberOFDronesInFomration; j++)
        {
            _droneID_type targetDrone = droneIDsInFormation[j];
            //printf("targetdrone:%d:\n", targetDrone);
            std::vector<_droneID_type>
                sourceDrones;
            bool validSources = false;
            sourceDrones.reserve(numberOFDronesInFomration);
            for (size_t i = 0; i < numberOFDronesInFomration; i++)
            {
                //std::cout << AdjacencyMatrix[i][j] << std::endl;
                if (AdjacencyMatrix[i][j])
                {
                    validSources = true;
                    //std::cout << (int)i << "," << (int)j << ":1" << std::endl;
                    sourceDrones.push_back(droneIDsInFormation[i]);
                }
            }
            
            if (validSources)
            {
                auto search = dataLinks.find(targetDrone);
                if (search != dataLinks.end())
                {

                    search->second = sourceDrones;
                }
                else
                {

                    dataLinks.emplace(targetDrone, sourceDrones);
                    nextSendTime.emplace(targetDrone, 0);
                }
            }
        }
        keys.reserve(dataLinks.size());
        for (auto const &dataLink : dataLinks)
            keys.push_back(dataLink.first);
        pthread_mutex_unlock(&communicationGraphlock);
    }
    template<typename data_type>
    void SetGraph(Configuration<data_type>& configuration)
    {

        _droneID_type droneIDsInFormation[MAXIMUM_CONSTELLATION_SIZE];
        int i = 0;
        for (auto data:configuration.dronesAndTypes)
        {
            droneIDsInFormation[i]=data.first;
            i++;
        }
        unsigned char numberOFDronesInFomration = configuration.dronesAndTypes.size();

        bool adjacencyMatrix[MAXIMUM_CONSTELLATION_SIZE][MAXIMUM_CONSTELLATION_SIZE];
        for (size_t i = 0; i < MAXIMUM_CONSTELLATION_SIZE; i++)
        {
            for (size_t j = 0; j < MAXIMUM_CONSTELLATION_SIZE; j++)
            {
                adjacencyMatrix[i][j] = false;
            }
            
        }
        
        typedef Eigen::Triplet<data_type> Triplet;
        for (auto triplet : configuration.communicationEntries)
        {
            //get row index in adjacency matrix
            size_t i = configuration.dronesAndTypes.find(triplet.row())->second.second;
            //get column index in adjacency matrix
            size_t j = configuration.dronesAndTypes.find(triplet.col())->second.second;
            adjacencyMatrix[i][j] = true;            
        }
        //std::cout << (int)numberOFDronesInFomration << std::endl;
        SetGraph(droneIDsInFormation, numberOFDronesInFomration, adjacencyMatrix);

    }

    /** GetSourceDrones
        Get targetted drones for a specific drones data.

        @param targetDroneID _droneID_type ID of target drone.
        @return std::vector<_droneID_type> Sourced drones.
        */
    std::vector<_droneID_type> GetSourceDrones(_droneID_type targetDroneID)
    {
        std::vector<_droneID_type> result;
        pthread_mutex_lock(&communicationGraphlock);
        auto search = dataLinks.find(targetDroneID);

        if (search != dataLinks.end())
        {
            result = search->second;
        }
        pthread_mutex_unlock(&communicationGraphlock);
        return result;
    }

        bool Empty()
        {
            bool empty = false;
            pthread_mutex_lock(&communicationGraphlock);
            empty = dataLinks.empty();
            pthread_mutex_unlock(&communicationGraphlock);
            return empty;
        }

        std::size_t Size()
        {
            std::size_t size = 0;
            pthread_mutex_lock(&communicationGraphlock);
            size = dataLinks.size();
            pthread_mutex_unlock(&communicationGraphlock);
            return size;
        }

        std::vector < _droneID_type> GetTargetDrones()
        {
            std::vector<_droneID_type> droneIDs;
            pthread_mutex_lock(&communicationGraphlock);
                droneIDs = keys;
            pthread_mutex_unlock(&communicationGraphlock);

            return droneIDs;
        }

        bool TimingCheck(_droneID_type targetDroneID)
        {
            bool send = false;
            auto search = nextSendTime.find(targetDroneID);

            if (search != nextSendTime.end())
            {
                send = CheckDataSendTiming(search->second, DATA_DISTRIBUTION_INTEVRAL_MS);
            }

            return send;
        }
};