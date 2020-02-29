#ifndef DERIVEDPLANTDATA_HPP
#define DERIVEDPLANTDATA_HPP
#include <iostream>
//For using Eigen Library for matrix calculations
#include <vector>


#include <Eigen/Dense>
#include "uav_constants.hpp"
#include "../Maths/math_helper.hpp"
template<typename data_type>
class DerivedPlantData{
        typedef std::vector< data_type > state_type;
            //declaring matrix and vector types used
        typedef Eigen::Matrix<data_type, 3, 1> vector_type;
        typedef Eigen::Matrix<data_type, 3, 3> matrix_type;
        public:

        data_type Va =0;
        data_type alpha = 0;
        data_type beta = 0;
        data_type chi = 0;
        data_type q_bar = 0;
        matrix_type DCMeb;
        data_type Ve[3]= {0,0,0};

        data_type sin_phi = 0;
        data_type cos_phi = 0;
        data_type sin_theta = 0;
        data_type cos_theta = 0;
        data_type sin_psi = 0;
        data_type cos_psi = 0;

        data_type tan_theta = 0;

        data_type Va_vector[3];
        
        
        DerivedPlantData(){

            DCMeb<<0,0,0,0,0,0,0,0,0;
            
            //std::cout<<"cocalledderi"<<std::endl;
            }
        ~DerivedPlantData(){}

        DerivedPlantData(state_type &y, const data_type (&wind_vector)[3]={0,0,0}):DerivedPlantData()
        {
            //vector_type Va_vector;
            Va_vector[0]= y[3]-wind_vector[0];
            Va_vector[1]= y[4]-wind_vector[1];
            Va_vector[2]= y[5]-wind_vector[2];
            Va=sqrt(pow(Va_vector[0],2)+pow(Va_vector[1],2)+pow(Va_vector[2],2));

            alpha = atan2(Va_vector[2],Va_vector[0]);
            beta = asin(Va_vector[1]/Va);

 
            q_bar = 0.5*Constants_C_rho*(sq(y[3])+sq(y[4])+sq(y[5]));

            //std::cout<<"phi:"<<y[6]<<"theta:"<<y[7]<<"psi:"<<y[8]<<std::endl;
            sin_phi = sin(y[6]);
            cos_phi = cos(y[6]);
            sin_theta = sin(y[7]);
            cos_theta = cos(y[7]);
            sin_psi = sin(y[8]);
            cos_psi = cos(y[8]);
            tan_theta = tan(y[7]);

            
            DCMeb<<cos_theta*cos_psi,-cos_phi*sin_psi+sin_phi*sin_theta*cos_psi,sin_phi*sin_psi+cos_phi*sin_theta*cos_psi,
                cos_theta*sin_psi, cos_phi*cos_psi+sin_phi*sin_theta*sin_psi, -sin_phi*cos_psi+cos_phi*sin_theta*sin_psi,
                -sin_theta, sin_phi*cos_theta, cos_phi*cos_theta;

            Ve[0] = y[5]*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta) - y[4]*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta) + y[3]*cos_psi*cos_theta;
            Ve[1] = y[4]*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - y[5]*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + y[3]*cos_theta*sin_psi;
            Ve[2] = y[5]*cos_phi*cos_theta - y[3]*sin_theta + y[4]*cos_theta*sin_phi;

            chi = atan2(Ve[1],Ve[0]);
                
        }


    };

    template <typename data_type>
    void RotateFromEarthToBody(data_type *X, const DerivedPlantData<data_type> &trajectoryDerivedData)
    {
        RotateFromEarthToBodyUsingSineCosineEuler(X,
                                                  trajectoryDerivedData.sin_phi,
                                                  trajectoryDerivedData.cos_phi,
                                                  trajectoryDerivedData.sin_theta,
                                                  trajectoryDerivedData.cos_theta,
                                                  trajectoryDerivedData.sin_psi,
                                                  trajectoryDerivedData.cos_psi);
    }

#endif