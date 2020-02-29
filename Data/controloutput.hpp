#pragma once

template<typename data_type>
class ControlOutput
{
    public:
    data_type F[3];
    data_type M[3];

    ControlOutput(){}
    ~ControlOutput(){}

    //copy constructor
    ControlOutput(const ControlOutput<data_type>& other )
    {
        //std::cout<<"cococalled"<<std::endl;
        this->F[0] = other.F[0];
        this->F[1] = other.F[1];
        this->F[2] = other.F[2];

        this->M[0] = other.M[0];
        this->M[1] = other.M[1];
        this->M[2] = other.M[2];
    }

    //copy assignment constructor
    ControlOutput<data_type> operator=(const ControlOutput<data_type>& other) 
    {
        //std::cout<<"cocacalled"<<std::endl;
        this->F[0] = other.F[0];
        this->F[1] = other.F[1];
        this->F[2] = other.F[2];

        this->M[0] = other.M[0];
        this->M[1] = other.M[1];
        this->M[2] = other.M[2];
        return *this;
    }

};