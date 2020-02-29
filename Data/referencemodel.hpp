#ifndef REFERENCE_MODEL_HPP
#define REFERENCE_MODEL_HPP

#include <vector>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Maths/math_helper.hpp>
#include <algorithm>
#include <DroneSim/derivedplantdata.hpp>
#include <pthread.h>
#include "protocol/byteconverter.hpp"
#include "protocol/packet.hpp"

template<typename data_type>
class ReferenceModel
{
    public:

        typedef Eigen::Matrix<data_type, 6, 1> vector6_type;

        typedef Eigen::Matrix<data_type, 3, 1>  vector_type;
        typedef Eigen::Matrix<data_type, 3, 3> matrix_type;

        typedef std::vector<data_type> state_type;

        typedef Eigen::SparseMatrix<data_type> SpMat; // declares a column-major sparse matrix type of double
        typedef Eigen::Triplet<data_type> Triplet;


    private:
        vector6_type Kp;
        vector6_type Kv;
        SpMat Am;
        SpMat Bm;
    pthread_mutex_t mutex_lock;
    
    ReferenceModel():Am(12,12),Bm(12,6)
    {
        pthread_mutex_init(&mutex_lock,NULL);
        
        //default values
        data_type Kp[6]= {100,100,100,100,100,100};
        data_type Kv[6]= {100,100,100,100,100,100};
        pthread_mutex_lock(&mutex_lock);
        data_type *Kp_arr = this->Kp.data();
        data_type *Kv_arr = this->Kv.data();

        if (Kp != nullptr)
        {
            std::copy(Kp, Kp + 6, Kp_arr);
        }
        //std::cout << this->Kp << std::endl;
        if (Kv != nullptr)
        {
            std::copy(Kv, Kv + 6, Kv_arr);
        }

        //for Am
        std::vector<Triplet> coefficients;
        coefficients.reserve(18);
        coefficients.push_back(Triplet(0, 6, 1));
        coefficients.push_back(Triplet(1, 7, 1));
        coefficients.push_back(Triplet(2, 8, 1));
        coefficients.push_back(Triplet(3, 9, 1));
        coefficients.push_back(Triplet(4, 10, 1));
        coefficients.push_back(Triplet(5, 11, 1));

        coefficients.push_back(Triplet(6, 0, -Kp_arr[0]));
        coefficients.push_back(Triplet(7, 1, -Kp_arr[1]));
        coefficients.push_back(Triplet(8, 2, -Kp_arr[2]));
        coefficients.push_back(Triplet(9, 3, -Kp_arr[3]));
        coefficients.push_back(Triplet(10, 4, -Kp_arr[4]));
        coefficients.push_back(Triplet(11, 5, -Kp_arr[5]));

        coefficients.push_back(Triplet(6, 6, -Kv_arr[0]));
        coefficients.push_back(Triplet(7, 7, -Kv_arr[1]));
        coefficients.push_back(Triplet(8, 8, -Kv_arr[2]));
        coefficients.push_back(Triplet(9, 9, -Kv_arr[3]));
        coefficients.push_back(Triplet(10, 10, -Kv_arr[4]));
        coefficients.push_back(Triplet(11, 11, -Kv_arr[5]));

        this->Am.setFromTriplets(coefficients.begin(), coefficients.end());
        //this->Am.makeCompressed();

        //for Bm
        coefficients.clear();
        coefficients.reserve(6);
        coefficients.push_back(Triplet(6, 0, 1));
        coefficients.push_back(Triplet(7, 1, 1));
        coefficients.push_back(Triplet(8, 2, 1));
        coefficients.push_back(Triplet(9, 3, 1));
        coefficients.push_back(Triplet(10, 4, 1));
        coefficients.push_back(Triplet(11, 5, 1));

        this->Bm.setFromTriplets(coefficients.begin(), coefficients.end());
        //this->Bm.makeCompressed();
        pthread_mutex_unlock(&mutex_lock);
    }

    public:
        ReferenceModel(ReferenceModel<data_type> const &) = delete;
        void operator=(ReferenceModel<data_type> const &) = delete;

        static ReferenceModel<data_type> &GetReferenceModel()
        {
            static ReferenceModel<data_type> instance; // Guaranteed to be destroyed.
                                                       // Instantiated on first use.
            return instance;
        }

        ~ReferenceModel()
        {
            pthread_mutex_destroy(&mutex_lock);
        }   
    vector6_type GetKv()
    {
        static vector6_type Kv_return;
        pthread_mutex_lock(&mutex_lock);
        Kv_return = Kv;
        pthread_mutex_unlock(&mutex_lock);
        return Kv_return;
    }

    vector6_type GetKp()
    {
        static vector6_type Kp_return;
        pthread_mutex_lock(&mutex_lock);
        Kp_return = Kp;
        pthread_mutex_unlock(&mutex_lock);
        return Kp_return;
    }

    void SetKpKv(Packet& packet)
    {
        Converter<Packet::data_type> converter;
        data_type KpIn[6] = {10,10,10,10,10,10};
        data_type KvIn[6] = {10,10,10,10,10,10};
        for (int i = 0; i < 6; i++)
        { //take care of endianess
            memcpy(converter.m_bytes, packet.data + (i * 4), 4);
            KpIn[i] = (data_type)converter.m_value;
        }
        for (int i = 6; i < 12; i++)
        { //take care of endianess
            memcpy(converter.m_bytes, packet.data + (i * 4), 4);
            KvIn[i-6] = (data_type)converter.m_value;
        }
        
        SetKpKv(KpIn,KvIn);
    }

    void SetKpKv(data_type *Kp, data_type *Kv)
    {
        pthread_mutex_lock(&mutex_lock);
        data_type * Kp_arr = this->Kp.data();
        data_type * Kv_arr = this->Kv.data();
        
        if(Kp!=nullptr){
            std::copy(Kp, Kp + 6, Kp_arr);
        }
        //std::cout << this->Kp << std::endl;
        if(Kv!=nullptr){
            std::copy(Kv, Kv + 6, Kv_arr);
        }      
        //std::cout << this->Kv << std::endl;
        //for Am
        std::vector<Triplet> coefficients;
        coefficients.reserve(18);
        coefficients.push_back(Triplet(0, 6, 1));
        coefficients.push_back(Triplet(1, 7, 1));
        coefficients.push_back(Triplet(2, 8, 1));
        coefficients.push_back(Triplet(3, 9, 1));
        coefficients.push_back(Triplet(4, 10, 1));
        coefficients.push_back(Triplet(5, 11, 1));

        coefficients.push_back(Triplet(6, 0, -Kp_arr[0]));
        coefficients.push_back(Triplet(7, 1, -Kp_arr[1]));
        coefficients.push_back(Triplet(8, 2, -Kp_arr[2]));
        coefficients.push_back(Triplet(9, 3, -Kp_arr[3]));
        coefficients.push_back(Triplet(10, 4, -Kp_arr[4]));
        coefficients.push_back(Triplet(11, 5, -Kp_arr[5]));

        coefficients.push_back(Triplet(6, 6, -Kv_arr[0]));
        coefficients.push_back(Triplet(7, 7, -Kv_arr[1]));
        coefficients.push_back(Triplet(8, 8, -Kv_arr[2]));
        coefficients.push_back(Triplet(9, 9, -Kv_arr[3]));
        coefficients.push_back(Triplet(10, 10, -Kv_arr[4]));
        coefficients.push_back(Triplet(11, 11, -Kv_arr[5]));
        std::cout << "Trying to Set Am" << std::endl;
        SpMat AmNew(12,12);
        //AmNew.reserve(18);
        AmNew.setFromTriplets(coefficients.begin(), coefficients.end());
        AmNew.makeCompressed();
        //std::cout << "AmNew:" <<AmNew<< std::endl;
        Am = AmNew;
        //std::cout << "Am:" << Am << std::endl;
        //std::cout << Am << std::endl;
        //for Bm
        coefficients.clear();
        coefficients.reserve(6);
        coefficients.push_back(Triplet(6, 0, 1));
        coefficients.push_back(Triplet(7, 1, 1));
        coefficients.push_back(Triplet(8, 2, 1));
        coefficients.push_back(Triplet(9, 3, 1));
        coefficients.push_back(Triplet(10, 4, 1));
        coefficients.push_back(Triplet(11, 5, 1));


        std::cout << "Trying to Set Bm" << std::endl;
        SpMat BmNew(12, 6);        
        //BmNew.reserve(6);
        BmNew.setFromTriplets(coefficients.begin(), coefficients.end());
        BmNew.makeCompressed();
        //std::cout << "BmNew:" <<BmNew<< std::endl;
        Bm = BmNew;
        //std::cout << "Bm:" << Bm << std::endl;
        //std::cout << Bm << std::endl;
        pthread_mutex_unlock(&mutex_lock);
    }

    SpMat GetAm()
    {
        SpMat Amreturn(12,12);
        pthread_mutex_lock(&mutex_lock);
        Amreturn = Am;
        pthread_mutex_unlock(&mutex_lock);

        return Amreturn;

    }

    SpMat GetBm()
    {
        SpMat Bmreturn(12,6);
        pthread_mutex_lock(&mutex_lock);
        Bmreturn = Bm;
        pthread_mutex_unlock(&mutex_lock);

        return Bmreturn;
    }

    inline vector6_type CalculateReferenceInput(
        state_type &trajectory,
        const DerivedPlantData<data_type> &trajectoryDerivedData,
        state_type &trajectoryAcceleration,
        state_type &trajectoryWbdot)
    {
        
        //using namespace Eigen;

        // //Mapping trajectory variables
        // Map<vector_type> Xed(&trajectory[0]);
        // vector_type Xd(trajectoryDerivedData.DCMeb.transpose() * Xed); //Modify
        // Map<vector_type> Vbd(&trajectory[3]);
        // Map<vector_type> eulerd(&trajectory[6]);
        // Map<vector_type> Wbd(&trajectory[9]);

        // Map<vector_type> accelerationd(&trajectoryAcceleration[0]);
        // Map<vector_type> Wbdotd(&trajectoryWbdot[0]);

        // q_type qd;
        // qd<<Xd,eulerd;

        // q_type qd_dot;
        // qd_dot<<Vbd,Wbd;

        // q_type qd_dotdot;
        // qd_dotdot<<accelerationd,Wbdotd;

        // q_type rTrajectory = qd_dotdot + ReferenceModel<data_type>::Kv.cwiseProduct(qd_dot) + ReferenceModel<data_type>::Kp.cwiseProduct(qd);

        data_type *Xed_arr = trajectory.data();
        data_type Xbd_arr[3];
        std::copy(Xed_arr, Xed_arr + 3, Xbd_arr);
        RotateFromEarthToBody(Xbd_arr, trajectoryDerivedData);
        data_type *Vbd_arr = Xed_arr + 3;
        data_type *eulerd_arr = Xed_arr + 6;
        data_type *Wbd_arr = Xed_arr + 9;

        data_type *acc_arr = trajectoryAcceleration.data();
        data_type *wbdot_arr = trajectoryWbdot.data();
        vector6_type rTrajectory;
        data_type *rT = rTrajectory.data();

        //using Kp,Kv - Lock and Use
        pthread_mutex_lock(&mutex_lock);
        data_type *Kp_arr = Kp.data();
        data_type *Kv_arr = Kv.data();      
        
        rT[0] = acc_arr[0] + Kv_arr[0] * Vbd_arr[0] + Kp_arr[0] * Xbd_arr[0];
        rT[1] = acc_arr[1] + Kv_arr[1] * Vbd_arr[1] + Kp_arr[1] * Xbd_arr[1];
        rT[2] = acc_arr[2] + Kv_arr[2] * Vbd_arr[2] + Kp_arr[2] * Xbd_arr[2];
        rT[3] = wbdot_arr[0] + Kv_arr[3] * Wbd_arr[0] + Kp_arr[3] * eulerd_arr[0];
        rT[4] = wbdot_arr[1] + Kv_arr[4] * Wbd_arr[1] + Kp_arr[4] * eulerd_arr[1];
        rT[5] = wbdot_arr[2] + Kv_arr[5] * Wbd_arr[2] + Kp_arr[5] * eulerd_arr[2];
        pthread_mutex_unlock(&mutex_lock);
        

        return rTrajectory;
    }

     void Calculate_aLeader(
        data_type* ai_arr, 
        data_type* rTrajectory_arr, 
        data_type* Xbi_arr, 
        data_type* euleri_arr,
        data_type* Vbi_arr,
        data_type* Wbi_arr
        )
    {
        //using Kp,Kv - Lock and Use
        pthread_mutex_lock(&mutex_lock);
        data_type *Kp_arr = Kp.data();
        data_type *Kv_arr = Kv.data();
        ai_arr[0] = rTrajectory_arr[0] - Kv_arr[0] * Vbi_arr[0] - Kp_arr[0] * Xbi_arr[0];
        ai_arr[1] = rTrajectory_arr[1] - Kv_arr[1] * Vbi_arr[1] - Kp_arr[1] * Xbi_arr[1];
        ai_arr[2] = rTrajectory_arr[2] - Kv_arr[2] * Vbi_arr[2] - Kp_arr[2] * Xbi_arr[2];
        ai_arr[3] = rTrajectory_arr[3] - Kv_arr[3] * Wbi_arr[0] - Kp_arr[3] * euleri_arr[0];
        ai_arr[4] = rTrajectory_arr[4] - Kv_arr[4] * Wbi_arr[1] - Kp_arr[4] * euleri_arr[1];
        ai_arr[5] = rTrajectory_arr[5] - Kv_arr[5] * Wbi_arr[2] - Kp_arr[5] * euleri_arr[2];
        pthread_mutex_unlock(&mutex_lock);
    }

    void Calculate_aFollower(
        data_type* a_arr, 
        data_type* eni_1_arr, 
        data_type* eni_2_arr)
    {
        pthread_mutex_lock(&mutex_lock);
        data_type *Kp_arr = Kp.data();
        data_type *Kv_arr = Kv.data();
        a_arr[0] = Kv_arr[0] * eni_2_arr[0] + Kp_arr[0] * eni_1_arr[0];
        a_arr[1] = Kv_arr[1] * eni_2_arr[1] + Kp_arr[1] * eni_1_arr[1];
        a_arr[2] = Kv_arr[2] * eni_2_arr[2] + Kp_arr[2] * eni_1_arr[2];
        a_arr[3] = Kv_arr[3] * eni_2_arr[3] + Kp_arr[3] * eni_1_arr[3];
        a_arr[4] = Kv_arr[4] * eni_2_arr[4] + Kp_arr[4] * eni_1_arr[4];
        a_arr[5] = Kv_arr[5] * eni_2_arr[5] + Kp_arr[5] * eni_1_arr[5];          
        pthread_mutex_unlock(&mutex_lock);

    }
    // template<typename data_type>
    // typename ReferenceModel<data_type>::vector6_type ReferenceModel<data_type>::Kp = [] {
    //     ReferenceModel<data_type>::vector6_type tmp;
    //     tmp << 50,50,50,50,50,50;
    //     return tmp;
    // }();

    // template<typename data_type>
    // typename ReferenceModel<data_type>::vector6_type ReferenceModel<data_type>::Kv = [] {
    //     ReferenceModel<data_type>::vector6_type tmp;
    //     tmp << 50,50,50,50,50,50;
    //     return tmp;
    // }();

//     template <typename data_type>
//     class ReferenceInput
//     {
//     public:
//     typedef Eigen::Matrix<data_type, 3, 1>  vector_type;
//     typedef Eigen::Matrix<data_type, 3, 3> matrix_type;
//     typedef Eigen::Matrix<data_type, 6, 1> q_type;
//     typedef std::vector<data_type> state_type;

//     static inline q_type Calculate(
//         state_type &trajectory,
//         const DerivedPlantData<data_type> &trajectoryDerivedData,
//         state_type &trajectoryAcceleration,
//         state_type &trajectoryWbdot,
//         ReferenceModel<data_type> referenceModel)
//     {
//         //using namespace Eigen;

//         // //Mapping trajectory variables
//         // Map<vector_type> Xed(&trajectory[0]);
//         // vector_type Xd(trajectoryDerivedData.DCMeb.transpose() * Xed); //Modify
//         // Map<vector_type> Vbd(&trajectory[3]);
//         // Map<vector_type> eulerd(&trajectory[6]);
//         // Map<vector_type> Wbd(&trajectory[9]);

//         // Map<vector_type> accelerationd(&trajectoryAcceleration[0]);
//         // Map<vector_type> Wbdotd(&trajectoryWbdot[0]);

//         // q_type qd;
//         // qd<<Xd,eulerd;

//         // q_type qd_dot;
//         // qd_dot<<Vbd,Wbd;

//         // q_type qd_dotdot;
//         // qd_dotdot<<accelerationd,Wbdotd;

//         // q_type rTrajectory = qd_dotdot + ReferenceModel<data_type>::Kv.cwiseProduct(qd_dot) + ReferenceModel<data_type>::Kp.cwiseProduct(qd);

//         data_type *Xed_arr = trajectory.data();
//         data_type Xbd_arr[3];
//         std::copy(Xed_arr, Xed_arr + 3, Xbd_arr);
//         RotateFromEarthToBody(Xbd_arr, trajectoryDerivedData);
//         data_type *Vbd_arr = Xed_arr + 3;
//         data_type *eulerd_arr = Xed_arr + 6;
//         data_type *Wbd_arr = Xed_arr + 9;

//         data_type *acc_arr = trajectoryAcceleration.data();
//         data_type *wbdot_arr = trajectoryWbdot.data();
//         data_type *Kp_arr = Kp.data();
//         data_type *Kv_arr = Kv.data();

//         data_type rT[6];
//         rT[0] = acc_arr[0] + Kv_arr[0] * Vbd_arr[0] + Kp_arr[0] * Xbd_arr[0];
//         rT[1] = acc_arr[1] + Kv_arr[1] * Vbd_arr[1] + Kp_arr[1] * Xbd_arr[1];
//         rT[2] = acc_arr[2] + Kv_arr[2] * Vbd_arr[2] + Kp_arr[2] * Xbd_arr[2];
//         rT[3] = wbdot_arr[0] + Kv_arr[3] * Wbd_arr[0] + Kp_arr[3] * eulerd_arr[0];
//         rT[4] = wbdot_arr[1] + Kv_arr[4] * Wbd_arr[1] + Kp_arr[4] * eulerd_arr[1];
//         rT[5] = wbdot_arr[2] + Kv_arr[5] * Wbd_arr[2] + Kp_arr[5] * eulerd_arr[2];

//         q_type rTrajectory;

//         std::copy(rT, rT + 6, rTrajectory.data());

//         return rTrajectory;

//         }

 };



#endif