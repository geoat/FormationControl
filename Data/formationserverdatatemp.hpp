
//to store address datatypes;
#include <netinet/in.h>
#include <map>

#include "formationconfig.hpp"

#include "protocol/packet.hpp"

#include "pthread.h"
#include "protocol/packetreceiver.hpp"

namespace std
{
template <>
struct less<struct sockaddr_in>
{
    bool operator()(const struct sockaddr_in &lhs, const struct sockaddr_in &rhs) const
    {
        return lhs.sin_addr.s_addr < rhs.sin_addr.s_addr;
    }
};
} // namespace std

class ServerSideDroneDataTemp
{
public:
    socklen_t addressLength;
    struct sockaddr_in address;
    unsigned int count;
    char buffer[1024];
};

//should be thread safe
class FormationServerDataTemp
{

    std::map<struct sockaddr_in, ServerSideDroneDataTemp> dataMatrix;

    pthread_mutex_t lock;

    int InitializeDafaults()
    {
        if (pthread_mutex_init(&lock, NULL) != 0)
        {
            printf("\n mutex init failed\n");
            return 1;
        }
    }

public:
    FormationServerDataTemp()
    {
        InitializeDafaults();
    }

    ~FormationServerDataTemp()
    {
        pthread_mutex_destroy(&lock);
    }

    void ReportData(ServerSideDroneDataTemp newData)
    {

        auto search = dataMatrix.find(newData.address);
        if (search != dataMatrix.end())
        {
            pthread_mutex_lock(&lock);
            search->second = newData;
            // if (search->second.count==0)
            //     search->second = newData;
            // else
            // {

            //     memcpy(search->second.buffer + search->second.count, newData.buffer, newData.count);
            //     search->second.count += newData.count;
            // }
            pthread_mutex_unlock(&lock);
        }
        else
        {
            pthread_mutex_lock(&lock);
            dataMatrix.emplace(newData.address, newData);
            pthread_mutex_unlock(&lock);
        }
    }

    int Size()
    {
        return dataMatrix.size();
    }

    void GetNextData(ServerSideDroneDataTemp &data)
    {
        static auto iterator = dataMatrix.begin();
        if (iterator == dataMatrix.end())
            iterator = dataMatrix.begin();
        pthread_mutex_lock(&lock);
        data = iterator->second;
        iterator->second.count = 0;
        pthread_mutex_unlock(&lock);
        iterator++;
    }
};