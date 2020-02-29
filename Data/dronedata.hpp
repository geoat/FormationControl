#ifndef DRONEDATA_HPP
#define DRONEDATA_HPP
#include <vector>
#include "../Types/formationtypes.hpp"
#include "../Timing/timing.hpp"
#include <iostream>
template <typename data_type>

struct DroneData
{
    _droneID_type droneID;
    typedef std::vector<data_type> state_type;
    state_type droneStates;
    state_type controlInput;
    _time_type lastReportedTime;
    DroneData()
    {
        droneStates = state_type(12,0);
        controlInput = state_type(6, 0);
    }

    bool IsTimeValid(_time_type timeGap)    {
        _time_type currentTime = GetCurrentTime_ms();

        return (currentTime <= (lastReportedTime + timeGap));
    }
};

#endif