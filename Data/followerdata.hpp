#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "../Environment/constants.hpp"
#include "Data/leaderdata.hpp"
template <typename data_type = double>
    class FollowerData : public LeaderData < data_type>
{
public:
    typedef LeaderData < data_type> Base;
    typedef Eigen::SparseMatrix<data_type> SpMat; // declares a column-major sparse matrix type of double
    typedef Eigen::Triplet<data_type> Triplet;
    typedef Eigen::Matrix < data_type, 6, 1, Eigen::DontAlign> eigen_vector6_type;

    SpMat xi_DnDiCi;
    SpMat xi_DnDi;

    FollowerData() : Base(), xi_DnDiCi(39,6), xi_DnDi(6,6)
    {
        Setxi_DnDi();
        Initializexi();
    }

    
    /* 
    n - current node
    i - source node */
    inline void Setxi(data_type *eulern, data_type *wbn, data_type *euleri, data_type *wbi)
    {
        Base::Setxi(eulern,wbn);
        Setxi_DnDiCi(wbi);
    }
    
   
private:
    inline void Setxi_DnDi()
    {
        xi_DnDi.setIdentity();
    }

    inline void Setxi_DnDiCi(data_type *wbi) 
    {
        data_type &p = *wbi;
        data_type &q = *(wbi + 1);
        data_type &r = *(wbi + 2);

        xi_DnDiCi.coeffRef(0, 1) = -r;
        xi_DnDiCi.coeffRef(1, 0) = r;

        xi_DnDiCi.coeffRef(0, 2) = q;
        xi_DnDiCi.coeffRef(2, 0) = -q;

        xi_DnDiCi.coeffRef(1, 2) = -p;
        xi_DnDiCi.coeffRef(2, 1) = p;

        xi_DnDiCi.coeffRef(3, 3) = -q;
        xi_DnDiCi.coeffRef(7, 3) = -r;

        xi_DnDiCi.coeffRef(10, 3) = p;
        xi_DnDiCi.coeffRef(14, 3) = q;

        xi_DnDiCi.coeffRef(15, 4) = r;
        xi_DnDiCi.coeffRef(18, 4) = -p;

        xi_DnDiCi.coeffRef(23, 4) = r;
        xi_DnDiCi.coeffRef(26, 4) = -p;

        xi_DnDiCi.coeffRef(27, 5) = -q;
        xi_DnDiCi.coeffRef(31, 5) = -r;

        xi_DnDiCi.coeffRef(34, 5) = p;
        xi_DnDiCi.coeffRef(38, 5) = q;
    }

    inline void Initializexi()
    {
        //xi_DnDiCi
        std::vector<Triplet> coefficients;

        coefficients.reserve(18);
        coefficients.push_back(Triplet(0, 1, 1));
        coefficients.push_back(Triplet(1, 0, 1));

        coefficients.push_back(Triplet(0, 2, 1));
        coefficients.push_back(Triplet(2, 0, 1));

        coefficients.push_back(Triplet(1, 2, 1));
        coefficients.push_back(Triplet(2, 1, 1));

        coefficients.push_back(Triplet(3, 3, 1));
        coefficients.push_back(Triplet(7, 3, 1));

        coefficients.push_back(Triplet(10, 3, 1));
        coefficients.push_back(Triplet(14, 3, 1));

        coefficients.push_back(Triplet(15, 4, 1));
        coefficients.push_back(Triplet(18, 4, 1));

        coefficients.push_back(Triplet(23, 4, 1));
        coefficients.push_back(Triplet(26, 4, 1));

        coefficients.push_back(Triplet(27, 5, 1));
        coefficients.push_back(Triplet(31, 5, 1));

        coefficients.push_back(Triplet(34, 5, 1));
        coefficients.push_back(Triplet(38, 5, 1));
        xi_DnDiCi.resize(39, 6);
        xi_DnDiCi.setFromTriplets(coefficients.begin(), coefficients.end());
        xi_DnDiCi.makeCompressed();

    }


};
