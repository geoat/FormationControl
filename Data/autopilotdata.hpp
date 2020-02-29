#pragma once
#include <pthread.h>

#include "../Data/formationdata.hpp"
#include "autopilotstates.hpp"
#include "node.hpp"
template<typename calc_data_type = double>
class AutoPilotData
{   
    
    public :
        //configuration data
        typedef double data_type;

           



        AutoPilotData(AutoPilotData<calc_data_type> const &) = delete;
        void operator=(AutoPilotData<calc_data_type> const &) = delete;
        ~AutoPilotData() 
        {
            pthread_mutex_destroy(&lock);
        }
        static AutoPilotData<calc_data_type> &GetAutoPilotData()
        {
            static AutoPilotData<calc_data_type> instance; 

            return instance;
        }

        void SetExitProgram()
        {
            pthread_mutex_lock(&lock);
                exitProgram = true;
            pthread_mutex_unlock(&lock);
        }

        bool ShouldExit()
        {
            bool result = false;
            pthread_mutex_lock(&lock);
                result = exitProgram;
            pthread_mutex_unlock(&lock);
            return result;
        }


        void SetAutoPilotState(AutoPilotStates stateIn)
        {
            pthread_mutex_lock(&lock);
                autoPilotState = stateIn;
            pthread_mutex_unlock(&lock);
        }

        void SetNodeType(NodeType typeTo)
        {
            pthread_mutex_lock(&lock);
                this->nodeType = typeTo;
            pthread_mutex_unlock(&lock);
        }

        bool RunControlAlgorthm()
        {
            bool result = false;
            pthread_mutex_lock(&lock);
            result = (autoPilotState==AutoPilotStates::active);
            pthread_mutex_unlock(&lock);
            return result;
        }

        NodeType GetNodeType()
        {
            NodeType nodeTypeResult;
            pthread_mutex_lock(&lock);
            nodeTypeResult = nodeType;
            pthread_mutex_unlock(&lock);
            return nodeTypeResult;
        }

    private:
        pthread_mutex_t lock;
        AutoPilotStates autoPilotState = AutoPilotStates::safe;
        NodeType nodeType = NodeType::Leader;

         //exit flag
        volatile bool exitProgram = false;
        AutoPilotData(){
            if (pthread_mutex_init(&lock, NULL) != 0)
            {
                printf("\n AckPacketQueueMatrix mutex init failed\n");
                exit(0);
            }
        }
        
};
