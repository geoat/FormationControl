#ifndef SATURATE_HPP
#define SATURATE_HPP
#include "math_helper.hpp"
template<typename data_type>
class Saturation
{
    public:
    data_type UL;
    data_type LL;
    template<typename data_type1>
    Saturation(data_type1 LL, data_type1 UL)
    {
        if(LL > UL)
            std::swap(LL, UL);
        
        this->UL = (data_type)UL;
        this->LL = (data_type)LL;   

    }

    void SaturateValue(data_type& value)
    {
        value = std::max(std::min(value,UL),LL);
    }
};

#endif