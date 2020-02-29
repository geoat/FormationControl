#ifndef PID_HELPER_HPP
#define PID_HELPER_HPP
#include "math_helper.hpp"
#include <cmath>
template<typename data_type>
class pid{
    private:
    data_type I_Sat;
    data_type P;
    data_type I;
    data_type D;
    data_type FF;
    pid(){}
    
    public:
    template<typename data_type0, typename data_type1, typename data_type2, typename data_type3,typename data_type4>
    pid(
        const data_type0 P,
        const data_type1 I,
        const data_type2 D,
        const data_type3 FF, 
        const data_type4 I_Sat 
        ):P((data_type)P), I((data_type)I), D((data_type)D), FF((data_type)FF), I_Sat((data_type)I_Sat)
    {

    }
    ~pid(){}

    inline data_type Calc_PID_unsat(
        const data_type& scaler,
        const data_type& reference_input,         
        const data_type& input,
        const data_type& state) const
    {


        data_type I_term = sat(state,I_Sat,-I_Sat);
        data_type P_term = P*scaler*reference_input;
        data_type D_term = D*sq(scaler)*(reference_input-180/M_PI*input);
        data_type FF_term = FF*scaler*reference_input;
        return (I_term +P_term+D_term+FF_term);

    }
};
#endif