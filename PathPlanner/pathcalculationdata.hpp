#ifndef PATH_FOLLOWER_DATA_HPP
#define PATH_FOLLOWER_DATA_HPP
#include "path.hpp"
#include "Maths/math_helper.hpp"
#include "../Data/vector3.hpp"
//forward declaration
//forward declaration
template<typename> class Path;
template<typename> class Line;
template<typename> class Orbit;

template<typename data_type>
class PathCalculationData{
    public:
    static inline void MatchDomain(
            data_type& chi_q, 
            const data_type& chi){

            while ((chi_q - chi) < -M_PI)
                chi_q = chi_q + 2*M_PI;

            while ((chi_q - chi) > M_PI)
                chi_q = chi_q - 2*M_PI;
        }
};

template<typename data_type>
class LineCalculationData: public PathCalculationData<data_type>
{
    public:
    typedef PathCalculationData<data_type> parent;
    Line<data_type> *line;
    data_type chi_d;
    data_type chi_q;
    data_type e_y;
    

    LineCalculationData( 
        Line<data_type>* line, 
        const DerivedPlantData<data_type>& derivedData,
        const Point<data_type> location)
    {
        this->line = line;
        chi_q = atan2(line->slope.y,line->slope.x);
        parent::MatchDomain(chi_q,derivedData.chi);

        e_y = -sin(chi_q)*(location.x-line->path_origin.x) + cos(chi_q)*(location.y-line->path_origin.y);

        chi_d = chi_q - LineVFParameters<data_type>::chi_inf*TWO_OVER_M_PI*atan(LineVFParameters<data_type>::line_k*e_y);

    }
    ~LineCalculationData(){}


};

template<typename data_type>
class OrbitCalculationData: public PathCalculationData<data_type>
{
    public:
    typedef PathCalculationData<data_type> parent;
    Orbit<data_type> *orbit;

    data_type chi_d;
    data_type gamma;
    data_type e_y;
    data_type d;

    OrbitCalculationData( 
        Orbit<data_type>* orbit, 
        const DerivedPlantData<data_type>& derivedData,
        const Point<data_type> location)
    {
        this->orbit = orbit;
        //std::cout<<"location.y:"<<location.y<<std::endl;
        //std::cout<<"location.x:"<<location.x<<std::endl;
        auto deltaVector = location - orbit->centre;
        //std::cout<<"deltaVector.y:"<<deltaVector.y<<std::endl;
        //std::cout<<"deltaVector.x:"<<deltaVector.x<<std::endl;

        gamma = atan2(deltaVector.y,deltaVector.x);
        
        d = deltaVector.norm();
        parent::MatchDomain(gamma,derivedData.chi);
        data_type d_tilde = d - orbit->radius;

        chi_d = gamma + orbit->direction*(M_PI/2 + atan(OrbitVFParameters<data_type>::k*(d_tilde)));
    


    }

};


#endif