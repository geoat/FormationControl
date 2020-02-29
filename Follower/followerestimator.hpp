#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "../Data/followerdata.hpp"
#include "../Data/referencemodel.hpp"
#include "../Maths/math_helper.hpp"
#include "../Data/estimationgains.hpp"
#include "../DroneSim/derivedplantdata.hpp"

template <typename data_type = double>
class FollowerEstimator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::vector<data_type> state_type;
    typedef Eigen::SparseMatrix<data_type> SpMat; // declares a column-major sparse matrix type of double
    typedef Eigen::Triplet<data_type> Triplet;
    typedef Eigen::Matrix<data_type, 12, 1> eigen_vector12_type;
    typedef Eigen::Matrix<data_type, 6, 1, Eigen::DontAlign> eigen_vector6_type_nonaligned;
    typedef Eigen::Matrix<data_type, 6, 1> eigen_vector6_type;
    typedef Eigen::Matrix<data_type, 3, 1> eigen_vector3_type;

    //Theta
    SpMat Theta_DnT;
    SpMat Theta_CnT;
    SpMat Theta_gnT;
    SpMat Theta_DnDiT;
    SpMat Theta_DnDiCiT;

    FollowerEstimator(FollowerData<data_type> &followerData,
                      data_type &integrationTime) : integrationTime(integrationTime),
                                                    followerData(followerData),
                                                    Theta_Dn_dotT(6, 6),
                                                    Theta_Cn_dotT(6, 15),
                                                    Theta_gn_dotT(6, 6),
                                                    Theta_DnDi_dotT(6, 6),
                                                    Theta_DnDiCi_dotT(6, 39),
                                                    Theta_DnT(6, 6),
                                                    Theta_CnT(6, 15),
                                                    Theta_gnT(6, 6),
                                                    Theta_DnDiT(6, 6),
                                                    Theta_DnDiCiT(6, 39),
                                                    Theta_DnT_1(6, 6),
                                                    Theta_CnT_1(6, 15),
                                                    Theta_gnT_1(6, 6),
                                                    Theta_DnDiT_1(6, 6),
                                                    Theta_DnDiCiT_1(6, 39)
    {
    }

    void CalculateDerivatives(Eigen::Ref<eigen_vector3_type> Xbi,
                              Eigen::Ref<eigen_vector3_type> Vbi,
                              Eigen::Ref<eigen_vector3_type> euleri,
                              Eigen::Ref<eigen_vector3_type> Wbi,
                              Eigen::Ref<eigen_vector3_type> Xbn,
                              Eigen::Ref<eigen_vector3_type> Vbn,
                              Eigen::Ref<eigen_vector3_type> eulern,
                              Eigen::Ref<eigen_vector3_type> Wbn,
                              Eigen::Ref<eigen_vector6_type> taui)
    {

        //calculate error eni_1
        eigen_vector6_type eni_1;
        eni_1 << Xbn - Xbi, eulern - euleri;

        //calculate error eni_2
        eigen_vector6_type eni_2;
        eni_2 << Vbn - Vbi, Wbn - Wbi;

        //calculate error eni
        eigen_vector12_type eni;
        eni << eni_1, eni_2;

        //calculate 
        eigen_vector6_type qi_dot, qn_dot;
        qn_dot << Vbn, Wbn;
        qi_dot << Vbi, Wbi;

        //Eigen::Map<eigen_vector6_type> al(followerData.al.data());

        Product_S_BmT_P_eFollower = estGain.GetS() * refModel.GetBm().transpose() * estGain.GetP() * eni;
        //std::cout << "****Product_S_BmT_P_eFollower****\n"<< Theta_Dn_dotT << std::endl;
        Theta_Dn_dotT = (Product_S_BmT_P_eFollower * followerData.al.transpose() * followerData.xi_Dn.transpose()).sparseView();
        Theta_Cn_dotT = (-Product_S_BmT_P_eFollower * qn_dot.transpose() * followerData.xi_Cn.transpose()).sparseView();
        Theta_gn_dotT = (-Product_S_BmT_P_eFollower * followerData.xi_gn.transpose()).sparseView();
        Theta_DnDi_dotT = (-Product_S_BmT_P_eFollower * taui.transpose() * followerData.xi_DnDi.transpose()).sparseView();
        Theta_DnDiCi_dotT = (Product_S_BmT_P_eFollower * qi_dot.transpose() * followerData.xi_DnDiCi.transpose()).sparseView();
    }

    void EstimateTheta(
        Eigen::Ref<eigen_vector3_type> Xbi,
        Eigen::Ref<eigen_vector3_type> Vbi,
        Eigen::Ref<eigen_vector3_type> euleri,
        Eigen::Ref<eigen_vector3_type> Wbi,
        Eigen::Ref<eigen_vector3_type> Xbn,
        Eigen::Ref<eigen_vector3_type> Vbn,
        Eigen::Ref<eigen_vector3_type> eulern,
        Eigen::Ref<eigen_vector3_type> Wbn,
        Eigen::Ref<eigen_vector6_type> taui)
    {
        CalculateDerivatives(
            Xbi,
            Vbi,
            euleri,
            Wbi,
            Xbn,
            Vbn,
            eulern,
            Wbn,
            taui);

        Theta_DnT = Theta_DnT_1;
        Theta_CnT = Theta_CnT_1;
        Theta_gnT = Theta_gnT_1;
        Theta_DnDiT = Theta_DnDiT_1;
        Theta_DnDiCiT = Theta_DnDiCiT_1;
        //std::cout << "****Theta_Di_dotT****\n"<< Theta_Dn_dotT << std::endl;
        //std::cout << "****Theta_Cn_dotT****\n" << Theta_Cn_dotT << std::endl;
        //std::cout << "****Theta_gn_dotT****\n" << Theta_gn_dotT << std::endl;

        Theta_DnT_1 += Theta_Dn_dotT * integrationTime;
        Theta_CnT_1 += Theta_Cn_dotT * integrationTime;
        Theta_gnT_1 += Theta_gn_dotT * integrationTime;
        Theta_DnDiT_1 += Theta_DnDi_dotT * integrationTime;
        Theta_DnDiCiT_1 += Theta_DnDiCi_dotT * integrationTime;
    }

private:
    FollowerData<data_type> &followerData;
    //Theta_derivatives
    SpMat Theta_Dn_dotT;
    SpMat Theta_Cn_dotT;
    SpMat Theta_gn_dotT;
    SpMat Theta_DnDi_dotT;
    SpMat Theta_DnDiCi_dotT;



    eigen_vector6_type Product_S_BmT_P_eFollower;

    SpMat Theta_DnT_1;
    SpMat Theta_CnT_1;
    SpMat Theta_gnT_1;
    SpMat Theta_DnDiT_1;
    SpMat Theta_DnDiCiT_1;

    data_type integrationTime;

    FormationEstimationGains<data_type> &estGain = FormationEstimationGains<data_type>::GetFormationEstimationGains();
    ReferenceModel<data_type> &refModel = ReferenceModel<data_type>::GetReferenceModel();
};