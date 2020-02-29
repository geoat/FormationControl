#ifndef MATH_HELPER_HPP
#define MATH_HELPER_HPP
#ifndef _USE_MATH_DEFINES
	#define _USE_MATH_DEFINES
#endif

#ifndef M_PI_SQR
    #define M_PI_SQR (M_PI*M_PI)
#endif

#ifndef TWO_OVER_M_PI
    #define TWO_OVER_M_PI (2/M_PI)
#endif

#define sq(x) pow(x,2)

#include <cmath>
#include <cstring>

template <typename T> 
inline int sign(T value) {
    return (T(0) < value) - (value < T(0));
}

// template <typename data_type, typename data_type1, typename commonType= typename std::common_type<data_type, data_type1>::type>
// typename std::common_type<data_type, data_type1>::type max(const data_type value1, const data_type1 value2)
// {
//    return (value1>value2)?value1:value2;

// }

// template <typename data_type, typename data_type1, typename commonType= typename std::common_type<data_type, data_type1>::type>
// typename std::common_type<data_type, data_type1>::type min(const data_type& value1, const data_type1 value2)
// {
//    return (value1<value2)?value1:value2;

// }


template <typename data_type, typename data_type1 >
inline data_type sat(const data_type& value, const data_type1 hi, const data_type1 lo)
{
    return std::max(std::min(value,(data_type)hi),(data_type)lo);
}

template <typename data_type>
inline void RotateFromEarthToBodyUsingSineCosineEuler(
    data_type *X_arr, 
    data_type sin_phi,
    data_type cos_phi,
    data_type sin_theta,
    data_type cos_theta,
    data_type sin_psi,
    data_type cos_psi
)
{

    data_type temp[3];
    temp[0] = X_arr[0] * cos_psi * cos_theta - X_arr[2] * sin_theta + X_arr[1] * cos_theta * sin_psi;
    temp[1] = X_arr[1] * (cos_phi * cos_psi + sin_phi * sin_psi * sin_theta) - X_arr[0] * (cos_phi * sin_psi - cos_psi * sin_phi * sin_theta) + X_arr[2] * cos_theta * sin_phi;
    X_arr[2] = X_arr[0] * (sin_phi * sin_psi + cos_phi * cos_psi * sin_theta) - X_arr[1] * (cos_psi * sin_phi - cos_phi * sin_psi * sin_theta) + X_arr[2] * cos_phi * cos_theta;

    X_arr[0] = temp[0];
    X_arr[1] = temp[1];
}

template <typename data_type>
inline void RotateFromEarthToBody(data_type *X_arr, const data_type *euler)
{
    data_type sin_phi = sin(euler[0]);
    data_type cos_phi = cos(euler[0]);
    data_type sin_theta = sin(euler[1]);
    data_type cos_theta = cos(euler[1]);
    data_type sin_psi = sin(euler[2]);
    data_type cos_psi = cos(euler[2]);
    RotateFromEarthToBodyUsingSineCosineEuler(
        X_arr,
        sin_phi,
        cos_phi,
        sin_theta,
        cos_theta,
        sin_psi,
        cos_psi
    );


}



#endif