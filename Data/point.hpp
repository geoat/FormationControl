#ifndef POINT_HPP
#define POINT_HPP
#include <cmath>

template<typename data_type>
class Point
{
    public:
    data_type x;
    data_type y;
    // constructors
    Point()
    {
    	this->x = 0; this->y = 0;
    }
    
    template<typename T>
    Point(const T x, const T y)
    {
    	this->x = (data_type)x; this->y = (data_type)y;
    }
    Point operator+(const Point& other) const
    {
    	return  Point((this->x+other.x),(this->y+other.y));
    }
    
    Point operator-(const Point& other) const
    {
    	return Point((this->x-other.x),(this->y-other.y));
    }

    template<typename T>
    Point operator*(const T& scalar) const
    {
    	return Point((this->x*scalar),(this->y*scalar));
    }

    template<typename T>
    Point operator/(const T& scalar) const
    {
    	return Point((this->x/scalar),(this->y/scalar));
    }



    
    Point& operator+=(const Point& other) 
    {
    	this->x = this->x + other.x;
        this->y = this->y + other.y;
        return *this;
    }
    Point& operator-=(const Point& other) 
    {
    	this->x = this->x - other.x;
        this->y = this->y - other.y;
        return *this;
    } 

    template<typename T>
    Point& operator*=(const T& scalar)
    {
    	this->x *= scalar;
        this->y *= scalar;
        return *this;
    }

    template<typename T>
    Point& operator/=(const T& scalar)
    {
    	this->x /= scalar;
        this->y /= scalar;
        return *this;
    }

    inline data_type dot(const Point& other)
    {
        return this->x*other.x+this->y*other.y;
    }

    inline data_type norm()
    {
        return sqrt(this->dot(*this));
    }

    Point& rotate(const data_type& angle) 
    { 
    	//new_x = old_x*cos(angle) - old_y*sin(angle)
        //new_y = old_x*sin(angle) + old_y*cos(angle)
        double old_x = this->x;
        
        this->x = this->x*cos(angle) - this->y*sin(angle);
        this->y = old_x*sin(angle) + this->y*cos(angle);
        return *this;

    }

    Point& rotate(const data_type&  angle, const Point&  other){ 
        //shifting the origin
       	*this -=other;
        
        //rotating by the given angle about the new origin
        this->rotate(angle);
        //Shifting origin back to the exact origin
        *this += other;
        return *this; 
    }


    Point unit()
    {
        return *this/(this->norm());
    }


    
};



#endif