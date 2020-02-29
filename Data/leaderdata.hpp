#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include "../Environment/constants.hpp"
template <typename data_type=double>
class LeaderData
{
    public:

    typedef Eigen::SparseMatrix<data_type> SpMat; // declares a column-major sparse matrix type of double
    typedef Eigen::Triplet<data_type> Triplet;
    typedef Eigen::Matrix<data_type, 6, 1, Eigen::DontAlign> eigen_vector6_type;
    

    SpMat xi_Dn;
    SpMat xi_Cn;
    SpMat xi_gn;

    eigen_vector6_type al;

    LeaderData() :  xi_Dn(6, 6), xi_Cn(15, 6), xi_gn(6, 1)
    {
        //al.reserve(6);
        Setxi_Dn();
        Initializexi();
    }

    

    


    virtual inline void Setxi(data_type *euler,data_type *wb) 
    {
        Setxi_Cn(wb);
        Setxi_gn(euler);
    }
    private:
    inline void Setxi_Cn(data_type *wb)
    {
        data_type &p = *wb;
        data_type &q = *(wb + 1);
        data_type &r = *(wb + 2);

        xi_Cn.coeffRef(1, 0 )= r;
        xi_Cn.coeffRef(0, 1 )= -r;

        xi_Cn.coeffRef(2, 0 )= -q;
        xi_Cn.coeffRef(0, 2 )= q;

        xi_Cn.coeffRef(2, 1 )= p;        
        xi_Cn.coeffRef(1, 2 )= -p;

        xi_Cn.coeffRef(4, 5 )= p;
        xi_Cn.coeffRef(5, 4 )= -p;

        xi_Cn.coeffRef(6, 5 )= -q;
        xi_Cn.coeffRef(8, 3 )= q;

        xi_Cn.coeffRef(9, 4 )= r;
        xi_Cn.coeffRef(10, 3)=  -r;

        xi_Cn.coeffRef(12, 4)=  -p;
        xi_Cn.coeffRef(13, 3)=  p;

        xi_Cn.coeffRef(13, 5)=  -r;
        xi_Cn.coeffRef(14, 4)=  r;

        // std::vector<Triplet> coefficients;
        // coefficients.reserve(16);
        // coefficients.push_back(Triplet(1, 0, r));
        // coefficients.push_back(Triplet(0, 1, -r));

        // coefficients.push_back(Triplet(2, 0, -q));
        // coefficients.push_back(Triplet(0, 2, q));

        // coefficients.push_back(Triplet(2, 1, p));        
        // coefficients.push_back(Triplet(1, 2, -p));

        // coefficients.push_back(Triplet(4, 5, p));
        // coefficients.push_back(Triplet(5, 4, -p));

        // coefficients.push_back(Triplet(6, 5, -q));
        // coefficients.push_back(Triplet(8, 3, q));

        // coefficients.push_back(Triplet(9, 4, r));
        // coefficients.push_back(Triplet(10, 3, -r));

        // coefficients.push_back(Triplet(12, 4, -p));
        // coefficients.push_back(Triplet(13, 3, p));

        // coefficients.push_back(Triplet(13, 5, -r));
        // coefficients.push_back(Triplet(14, 4, r));
        // xi_Cn.resize(15,6);
        // xi_Cn.setFromTriplets(coefficients.begin(), coefficients.end());
        
        //std::cout << xi_Cn << std::endl;
    }

    inline void Setxi_gn(data_type *euler)
    {
        data_type &phi = *euler;
        data_type &theta = *(euler + 1);
        data_type &psi = *(euler + 2);

        xi_gn.coeffRef(0, 0) =  -sin(theta) * Constants_g;
        xi_gn.coeffRef(1, 0) =  +sin(phi) * cos(theta) * Constants_g;

        xi_gn.coeffRef(2, 0) =  +cos(phi) * cos(theta) * Constants_g;


        
        // std::vector<Triplet> coefficients;
        // coefficients.reserve(3);
        // coefficients.push_back(Triplet(0, 0, sin(theta) * Constants_g));
        // coefficients.push_back(Triplet(1, 0, -sin(phi) * cos(theta) * Constants_g));

        // coefficients.push_back(Triplet(2, 0, -cos(phi) * cos(theta) * Constants_g));
        // xi_gn.resize(6,1);
        // xi_gn.setFromTriplets(coefficients.begin(), coefficients.end());
        //std::cout<<xi_gn<<std::endl;
    }

    
    inline void Setxi_Dn()
    {
        xi_Dn.setIdentity();
        
    }
    inline void Initializexi()
    {
        //xi_Cn
        std::vector<Triplet> coefficients;

        coefficients.reserve(16);
        coefficients.push_back(Triplet(1, 0, 1));
        coefficients.push_back(Triplet(0, 1, 1));

        coefficients.push_back(Triplet(2, 0, 1));
        coefficients.push_back(Triplet(0, 2, 1));

        coefficients.push_back(Triplet(2, 1, 1));
        coefficients.push_back(Triplet(1, 2, 1));

        coefficients.push_back(Triplet(4, 5, 1));
        coefficients.push_back(Triplet(5, 4, 1));

        coefficients.push_back(Triplet(6, 5, 1));
        coefficients.push_back(Triplet(8, 3, 1));

        coefficients.push_back(Triplet(9, 4, 1));
        coefficients.push_back(Triplet(10, 3, 1));

        coefficients.push_back(Triplet(12, 4, 1));
        coefficients.push_back(Triplet(13, 3, 1));

        coefficients.push_back(Triplet(13, 5, 1));
        coefficients.push_back(Triplet(14, 4, 1));
        xi_Cn.resize(15, 6);
        xi_Cn.setFromTriplets(coefficients.begin(), coefficients.end());
        xi_Cn.makeCompressed();

        //xi_gn

        coefficients.clear();
        coefficients.reserve(3);
        coefficients.push_back(Triplet(0, 0, 1));
        coefficients.push_back(Triplet(1, 0, 1));

        coefficients.push_back(Triplet(2, 0, 1));
        xi_gn.resize(6, 1);
        xi_gn.setFromTriplets(coefficients.begin(), coefficients.end());
        xi_gn.makeCompressed();
    }
};
