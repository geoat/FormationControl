#ifndef PHASE_WRAP_HPP
#define PHASE_WRAP_HPP
#include <exception>
template <typename data_type>
class phase_wrap
{
    data_type UL = 0;
    data_type LL = 0;
    data_type range_size = 0;
    public:
    phase_wrap(){}
    ~phase_wrap(){}
    template <typename data_type1>
    phase_wrap(data_type1 UL, data_type1 LL)
    {
        if(LL > UL)
            std::swap(LL, UL);
        range_size =(data_type)(UL-LL);
        this->UL = (data_type)UL;
        this->LL = (data_type)LL;

        if(range_size==0)
            throw "there is no gap between UL and LL";

    }
    template <typename data_type1>
    inline data_type wrap(data_type1 value)
    {
        value-=LL;
        return value - (range_size * std::floor(value/range_size)) + LL;
    }

    template <typename data_type1>
    void wrapVector3(data_type1* value)
    {
        value[0] = wrap(value[0]);
        value[1] = wrap(value[1]);
        value[2] = wrap(value[2]);
    }



};

/* double boundBetween(double val, double lowerBound, double upperBound){


   
   return val - (rangeSize * std::floor(val/rangeSize)) + lowerBound;
} */
#endif