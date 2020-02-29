#pragma once

// To use time library of C
#include <sys/time.h>
#include <unistd.h>
#include "../Types/formationtypes.hpp"

#define CLIENT_MSG_TIME_PERIOD_MS 100
#define CLIENT_MSG_CONNECTION_LOST_TIME_PERIOD_MS (CLIENT_MSG_TIME_PERIOD_MS * 10)
#define CLIENT_MSG_TIME_PERIOD_US (CLIENT_MSG_TIME_PERIOD_MS * 1000)
#define CLIENT_MSG_TIME_PERIOD_CLOCK_TICKS (CLIENT_MSG_TIME_PERIOD_MS * CLOCKS_PER_SECOND / 1000)
#define CONFIGURATION_TO_CLIENT_TIMING 541

_time_type GetCurrentTime_us()
{
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    _time_type currrenTime_ms = (currentTime.tv_sec * 1000000 + currentTime.tv_usec);
    return currrenTime_ms;
}

_time_type GetCurrentTime_ms()
{
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    _time_type currrenTime_ms = (currentTime.tv_sec * 1000 + currentTime.tv_usec / 1000);
    return currrenTime_ms;
}

inline bool CheckDataSendTiming(long unsigned int& nextDataSendTime_ms, long unsigned int Time_GAP_MS)
{

    _time_type currentDataSendTime_ms = GetCurrentTime_ms();
    //printf("currentDataSendTime_ms:%u\n", currentDataSendTime_ms);
    if (nextDataSendTime_ms < currentDataSendTime_ms)
    {
        //printf("currentDataSendTime_ms:%u\n", currentDataSendTime_ms);
        //printf("CLIENT_MSG_TIME_PERIOD_MS:%d\n", CLIENT_MSG_TIME_PERIOD_MS);
        nextDataSendTime_ms = currentDataSendTime_ms + Time_GAP_MS;
        return true;
    }
    return false;
}

