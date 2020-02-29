#pragma once
#include <vector>
#include "pathtype.hpp"
#include "point.hpp"
template <typename data_type = double>
class PathPlannerData
{
public:
    PathType pathType;
    data_type radius;
    Point<data_type> orbitCentre;
    Point<data_type> lineOrigin;
    Point<data_type> lineSlope;
    std::vector<Point<data_type>> wayPoints;
};