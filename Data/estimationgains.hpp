#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "../protocol/packet.hpp"
#include <pthread.h>
#include <vector>
#include <iostream>


template <typename data_type = double>
class FormationEstimationGains
{
public:

    typedef Eigen::SparseMatrix<data_type> SpMat; // declares a column-major sparse matrix type of double
    typedef Eigen::Triplet<data_type> Triplet;

    ~FormationEstimationGains()
    {
        pthread_mutex_destroy(&mutex_lock);
    }

    static FormationEstimationGains<data_type> &GetFormationEstimationGains()
    {
        static FormationEstimationGains<data_type> instance; // Guaranteed to be destroyed.
                                                             // Instantiated on first use.
        return instance;
    }

    SpMat GetS()
    {
        SpMat Sout;
        pthread_mutex_lock(&mutex_lock);
        Sout = S;
        pthread_mutex_unlock(&mutex_lock);
        return Sout;
    }

    SpMat GetP()
    {
        SpMat Pout;
        pthread_mutex_lock(&mutex_lock);
        Pout = P;
        pthread_mutex_unlock(&mutex_lock);
        return Pout;
    }

    void SetS(Packet& packet)
    {
        Converter<Packet::data_type> converter;
        data_type Sarray[6][6] = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}};
        for (size_t i = 0; i < packet.dataLength; i=i+6)
        {
            memcpy(converter.m_bytes, packet.data + i + 2, 4);
            //printf("i:%u,i+1:%u\n",i,i+1);
            //printf("row:%u,col+1:%u\n", packet.data[i], packet.data[i + 1]);
            Sarray[packet.data[i]][packet.data[i + 1]] = (data_type)converter.m_value;
        }
        SetS(Sarray);
        return;
        
    }
    void SetP(Packet &packet)
    {
        Converter<Packet::data_type> converter;
        data_type Parray[12][12] = 
        {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
        };
        for (size_t i = 0; i < packet.dataLength; i = i + 6)
        {
            memcpy(converter.m_bytes, packet.data + i + 2, 4);
            //printf("i:%u,i+1:%u\n",i,i+1);
            //printf("row:%u,col+1:%u\n", packet.data[i], packet.data[i + 1]);
            Parray[packet.data[i]][packet.data[i + 1]] = (data_type)converter.m_value;
        }
        SetP(Parray);
        return;
    }

private:

    FormationEstimationGains() : S(6, 6), P(12, 12)
    {
        pthread_mutex_init(&mutex_lock, NULL);

        data_type Sarray[6][6] =
            {
                {100, 0, 0, 0, 0, 0},
                {0, 100, 0, 0, 0, 0},
                {0, 0, 100, 0, 0, 0},
                {0, 0, 0, 100, 0, 0},
                {0, 0, 0, 0, 100, 0},
                {0, 0, 0, 0, 0, 100}};

        data_type Parray[12][12] =
            {
                {100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100},
            };
        SetS(Sarray);
        SetP(Parray);
    }

    inline void SetS(data_type (&Sarray)[6][6])
    {
        std::vector<Triplet> coefficients;
        coefficients.reserve(36);

        for (size_t i = 0; i < 6; i++)
        {
            for (size_t j = 0; j < 6; j++)
            {
                if (Sarray[i][j] != 0)
                    coefficients.push_back(Triplet(i, j, Sarray[i][j]));
            }
        }
        SpMat SNew(6,6);
        SNew.setFromTriplets(coefficients.begin(), coefficients.end());
        pthread_mutex_lock(&mutex_lock);        
        S = SNew;
        //std::cout<<S<<std::endl;
        pthread_mutex_unlock(&mutex_lock);
    }

    inline void SetP(data_type (&Parray)[12][12])
    {
        std::vector<Triplet> coefficients;
        coefficients.reserve(144);

        for (size_t i = 0; i < 12; i++)
        {
            for (size_t j = 0; j < 12; j++)
            {
                if (Parray[i][j] != 0)
                    coefficients.push_back(Triplet(i, j, Parray[i][j]));
            }
        }
        pthread_mutex_lock(&mutex_lock);
        P.setFromTriplets(coefficients.begin(), coefficients.end());
        //std::cout<<P<<std::endl;
        pthread_mutex_unlock(&mutex_lock);
    }
    

    SpMat S;
    SpMat P;

    pthread_mutex_t mutex_lock;

};