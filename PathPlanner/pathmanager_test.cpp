
#include <iostream>
#define PATHTYPE_HPP_TEST
#include "path.hpp"
#include <pathmanager.hpp>
#include <type_traits>
#include <memory>

// template<typename T,typename U>
// typename std::enable_if<!(std::is_same<U,Line<T>>::value || std::is_same<U,Orbit<T>>::value), Point<T> >::type GetNextLocation(Point<T>& location, const U& mission)
// {
//     return location;
// }


// template<typename T,typename U>
// typename std::enable_if<std::is_same<U,Line<T>>::value, Point<T> >::type GetNextLocation(Point<T>& location, const U& mission)
// {
//     std::cout<<"Line"<<std::endl;
//     location = location+mission.slope*0.0001;
//     return location;
// }

// template<typename T,typename U>
// typename std::enable_if<std::is_same<U,Orbit<T>>::value, Point<T> >::type GetNextLocation(Point<T>& location, const U& mission)
// {
//     std::cout<<"Orbit"<<std::endl;
//     location.rotate(0.00001);
//     return location;
// }




int main()
{
    //Testing way class
    auto point1 = Point<float>(0,0);
    auto point2 = Point<float>(500,500);
    auto point3 = Point<float>(500,0);
    auto point4 = Point<float>(0,500);
    auto point5 = Point<float>(0,0);

    Way<float> way(0.0025,30,point1,point2,point3,point4,point5);
    
    PathManager<float> pathmanager(0.0025);
    pathmanager.SetPathAsLine(point1,point2);

    pathmanager.SetPathAsOrbit(30,point2);

    pathmanager.SetPathAsWay(30,point1,point2,point3,point4,point5);

    Point<float> location(1,1);
    
    while(!(fabs(location.x)<=0.1 && fabs(location.y)<=0.1))
    {
        Path<float>* mission;
       
        mission = pathmanager.path->GetMission(location);

        
        mission->GetNextLocation(location);
        std::cout<<location.x<<","<<location.y<<std::endl;
        //break;
    }

    // PathManager<float> pathmanager;
    // pathmanager.SetPathAsLine(point1,point2);
    // std::cout<<"------"<<std::endl;
    // pathmanager.SetPathAsOrbit(30,point2);
    // std::cout<<"------"<<std::endl;
    // pathmanager.SetPathAsWay(30,point2,point1,point1);

    return 0;
}