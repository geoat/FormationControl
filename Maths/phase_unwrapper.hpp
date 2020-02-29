#ifndef PHASE_UNWRAPPER_HPP
#define PHASE_UNWRAPPER_HPP
#include <cmath>
template<typename data_type,bool deg>
class phase_unwrapper;

template<typename data_type>
class phase_unwrapper<data_type,true>
{
    public:
    data_type offset = 0;
    data_type previous_value = 0;
    bool called = false;
    data_type unwrap(const data_type& value){
        if(called){
            data_type unwrapped_value = previous_value + angle_norm(value - previous_value);
            offset =  offset + unwrapped_value - value; 
            previous_value = value;
            return (value + offset);
        }
        else
        {
            previous_value = value;
            called = true;
            return value;
        }
    }  


    data_type unwrap_temp(const data_type& value) const{
        if(called){
            data_type unwrapped_value = previous_value + angle_norm(value - previous_value);
            data_type temp_offset =  offset + unwrapped_value - value; 
            
            return (value + temp_offset);
        }
        else
        {
            return value;
        }
    }  
    
      

    private:
    data_type angle_norm(data_type x) const{

        x = fmod(x + 180, 360);
        if (x < 0)
            x = x + 360;
        return (x - 180);
    }
    


};
#endif