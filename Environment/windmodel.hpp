#ifndef WINDMODEL_HPP
#define WINDMODEL_HPP
#include <memory>
#include "../Maths/math_helper.hpp"
#include "../Data/vector3.hpp"


template<typename data_type>
class WindData{
    public: 
    Vector3<data_type> ConstantComponent;
    Vector3<data_type> TimeVaryingComponent;
    Vector3<data_type> DynamicConstant;
    Vector3<data_type> TotalWind;
};

template<typename data_type, bool simulation=false>
class WindModel
{
    public:
    //Constant Component
    static Vector3<data_type> ConstantComponent;

    //time varying component
    static data_type frequency_x;    
    static data_type frequency_y;

    static data_type amplitude_x;
    static data_type amplitude_y;

    WindModel()
    {

    }
    ~WindModel()
    {

    }

    Vector3<data_type> GetTimeVaryingComponent(const data_type& time) const
    {
        Vector3<data_type> result;
        result.Value[0] = amplitude_x*sin(2*M_PI*frequency_x*time);
        result.Value[1] = amplitude_y*sin(2*M_PI*frequency_y*time);
        result.Value[2] = 0;
        return result;
    }

};

template<typename data_type>
class WindModel<data_type,true>: public WindModel<data_type,false>
{
    public:
    typedef WindModel<data_type,false> parent;

    //wind total
    WindData<data_type> GetWindData(const data_type& time)
    {
        WindData<data_type> windData;

        windData.ConstantComponent = parent::ConstantComponent;
        windData.TimeVaryingComponent = parent::GetTimeVaryingComponent(time);
        windData.TotalWind = windData.ConstantComponent + windData.TimeVaryingComponent + windData.DynamicConstant;


        return windData;
    }



};


template<typename T, bool simulation>
Vector3<T> WindModel<T,simulation>::ConstantComponent = {0,0,0};//{1,1,0.1};

template<typename T,bool simulation>
T WindModel<T,simulation>::frequency_x = 1;

template<typename T,bool simulation>
T WindModel<T,simulation>::frequency_y = 1;

template<typename T,bool simulation>
T WindModel<T,simulation>::amplitude_x = 0;

template<typename T,bool simulation>
T WindModel<T,simulation>::amplitude_y = 0;

#endif