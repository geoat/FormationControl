#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "../Data/leaderdata.hpp"
#include "../Data/referencemodel.hpp"
#include "../Maths/math_helper.hpp"
#include "../Data/estimationgains.hpp"
#include "../DroneSim/derivedplantdata.hpp"

template <typename data_type = double>
class LeaderEstimator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::vector<data_type> state_type;
    typedef Eigen::SparseMatrix<data_type> SpMat; // declares a column-major sparse matrix type of double
    typedef Eigen::Triplet<data_type> Triplet;
    typedef Eigen::Matrix<data_type, 12, 1> eigen_vector12_type;
    typedef Eigen::Matrix<data_type, 6, 1, Eigen::DontAlign> eigen_vector6_type;
    typedef Eigen::Matrix<data_type, 3, 1> eigen_vector3_type;

    //Theta
    SpMat Theta_DT;
    SpMat Theta_CT;
    SpMat Theta_gT;

    LeaderEstimator(LeaderData<data_type> &leaderData,
                    data_type &integrationTime) : integrationTime(integrationTime),
                                                  leaderData(leaderData),
                                                  Theta_D_dotT(6, 6),
                                                  Theta_C_dotT(6, 15),
                                                  Theta_g_dotT(6, 6),
                                                  Theta_DT(6, 6),
                                                  Theta_CT(6, 15),
                                                  Theta_gT(6, 6),
                                                  Theta_DT_1(6, 6),
                                                  Theta_CT_1(6, 15),
                                                  Theta_gT_1(6, 6)
    {
    }

    void CalculateDerivatives(Eigen::Ref<eigen_vector3_type> Xbm,
                              Eigen::Ref<eigen_vector3_type> Vbm,
                              Eigen::Ref<eigen_vector3_type> eulerm,
                              Eigen::Ref<eigen_vector3_type> Wbm,
                              Eigen::Ref<eigen_vector3_type> Xbl,
                              Eigen::Ref<eigen_vector3_type> Vbl,
                              Eigen::Ref<eigen_vector3_type> eulerl,
                              Eigen::Ref<eigen_vector3_type> Wbl

    )
    {

        //calculate error
        eigen_vector12_type errorLeader;
        errorLeader << Xbl - Xbm, eulerl - eulerm, Vbl - Vbm, Wbl - Wbm;

        //calculate q,qdot of Leader
        eigen_vector6_type ql, ql_dot;
        ql << Xbl, eulerl;
        ql_dot << Vbl, Wbl;
        //Eigen::Map<eigen_vector6_type> al(leaderData.al.data());

        Product_S_BmT_P_eLeader = -estGain.GetS() * refModel.GetBm().transpose() * estGain.GetP() * errorLeader;
        //std::cout << "****Product_S_BmT_P_eLeader****\n"<< Theta_D_dotT << std::endl;
        Theta_D_dotT = (Product_S_BmT_P_eLeader * leaderData.al.transpose() * leaderData.xi_Dn.transpose()).sparseView();
        Theta_C_dotT = (Product_S_BmT_P_eLeader * ql_dot.transpose() * leaderData.xi_Cn.transpose()).sparseView();
        Theta_g_dotT = (Product_S_BmT_P_eLeader * leaderData.xi_gn.transpose()).sparseView();
    }

    void EstimateTheta(
        Eigen::Ref<eigen_vector3_type> Xbm,
        Eigen::Ref<eigen_vector3_type> Vbm,
        Eigen::Ref<eigen_vector3_type> eulerm,
        Eigen::Ref<eigen_vector3_type> Wbm,
        Eigen::Ref<eigen_vector3_type> Xbl,
        Eigen::Ref<eigen_vector3_type> Vbl,
        Eigen::Ref<eigen_vector3_type> eulerl,
        Eigen::Ref<eigen_vector3_type> Wbl)
    {
        CalculateDerivatives(
            Xbm,
            Vbm,
            eulerm,
            Wbm,
            Xbl,
            Vbl,
            eulerl,
            Wbl);

        Theta_DT = Theta_DT_1;
        Theta_CT = Theta_CT_1;
        Theta_gT = Theta_gT_1;
        //std::cout << "****Theta_D_dotT****\n"<< Theta_D_dotT << std::endl;
        //std::cout << "****Theta_C_dotT****\n" << Theta_C_dotT << std::endl;
        //std::cout << "****Theta_g_dotT****\n" << Theta_g_dotT << std::endl;

        Theta_DT_1 += Theta_D_dotT * integrationTime;
        Theta_CT_1 += Theta_C_dotT * integrationTime;
        Theta_gT_1 += Theta_g_dotT * integrationTime;
    }

private:
    LeaderData<data_type> &leaderData;
    //Theta_derivatives
    SpMat Theta_D_dotT;
    SpMat Theta_C_dotT;
    SpMat Theta_g_dotT;
    eigen_vector6_type Product_S_BmT_P_eLeader;

    SpMat Theta_DT_1;
    SpMat Theta_CT_1;
    SpMat Theta_gT_1;

    data_type integrationTime;

    FormationEstimationGains<data_type> &estGain = FormationEstimationGains<data_type>::GetFormationEstimationGains();
    ReferenceModel<data_type> &refModel = ReferenceModel<data_type>::GetReferenceModel();
};