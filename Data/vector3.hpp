#ifndef VECTOR3_HPP
#define VECTOR3_HPP
#include <memory>


template<typename data_type>
struct Vector3
{
    public:
    data_type Value[3];
    data_type &x = Value[0];
    data_type &y = Value[1];
    data_type &z = Value[2];
    Vector3()
    {
        this->Value[0] = 0;
        this->Value[1] = 0;
        this->Value[2] = 0;
    }
    Vector3(const Vector3<data_type>& other)
    {
        this->Value[0] = other.Value[0];
        this->Value[1] = other.Value[1];
        this->Value[2] = other.Value[2];
    }

    Vector3(const data_type* other)
    {
        this->Value[0] = other[0];
        this->Value[1] = other[1];
        this->Value[2] = other[2];
    }

    Vector3(std::initializer_list<data_type> list)
    {
        std::uninitialized_copy(list.begin(),list.end(), Value);
    }



    Vector3<data_type> operator=(const Vector3<data_type>& other)
    {
        this->Value[0] = other.Value[0];
        this->Value[1] = other.Value[1];
        this->Value[2] = other.Value[2];

        return *this;
    }
    ~Vector3(){}
    inline Vector3<data_type> operator+(const Vector3<data_type>& other) const
    {
        Vector3<data_type> result;

        result.Value[0] = this->Value[0] + other.Value[0];
        result.Value[1] = this->Value[1] + other.Value[1];
        result.Value[2] = this->Value[2] + other.Value[2];
        return result;
    }

    inline Vector3<data_type> operator-(const Vector3<data_type>& other) const
    {
        Vector3<data_type> result;
        result.Value[0] = this->Value[0] - other.Value[0];
        result.Value[1] = this->Value[1] - other.Value[1];
        result.Value[2] = this->Value[2] - other.Value[2];
        return result;
    }

    
    inline void set(const data_type& x,const data_type& y, const data_type& z)
    {
        this->Value[0] = x;
        this->Value[1] = y;
        this->Value[2] = z;
    }
    inline data_type dot(const Vector3<data_type>& other) const
    {
        return x*other.x+y*other.y+z*other.z;
    }
    inline data_type norm() const
    {
        return sqrt(this->dot(*this));
    }      
    

};

#endif