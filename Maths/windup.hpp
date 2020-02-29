#ifndef WINDUP_H
#define WINDUP_H
#include <cmath>
template<typename data_type>
class windup
{
    public:
    data_type unsat_signal = 0;
    data_type HL = 0;
    data_type LL = 0;
    windup(){}
    ~windup(){}
    template<typename limits_type>
    windup(limits_type HL,limits_type LL):HL((data_type)HL),LL((data_type)LL)
    {}
    data_type get_value(const data_type& value) const
    {
        
        return (unsat_signal>HL)?(std::min(0.0,value)):((unsat_signal<LL)?(std::max(0.0,value)):value);
    }    
};

#endif