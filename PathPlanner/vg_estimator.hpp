#ifndef VG_ESTIMATOR_HPP
#define VG_ESTIMATOR_HPP
#define TWO_OVER_M_PI (2/M_PI)
#include <cmath>
#include <memory>

#include "DroneSim/derivedplantdata.hpp"
#include "../Data/point.hpp"
#include "path.hpp"
#include "Maths/math_helper.hpp"
#include "vf_parameters.hpp"
#include "vg_integrator.hpp"
#include "Environment/windmodel.hpp"
#include "pathcalculationdata.hpp"

//forward declaration
template<typename> class Path;
template<typename> class Line;
template<typename> class Orbit;

template<typename data_type>
class VgEstimator
{
    public:
        typedef std::vector<data_type> state_type;
        state_type Vg_hat;
        
        data_type Ts = 0;
        data_type t = 0;
    virtual data_type GetVgHat(
        PathCalculationData<data_type> *PathCalculationData,
        const DerivedPlantData<data_type>& derivedData,
        const Point<data_type> location, 
        const Vector3<data_type>& Vb
        ) = 0 ;

        

        virtual void operator()(state_type &y , state_type &yp , data_type t )  const = 0;



};


template<typename data_type>
class LineVgEstimator : public VgEstimator<data_type>
{
    public:
    typedef VgEstimator<data_type> parent;
    typedef typename parent::state_type state_type;
    LineCalculationData<data_type> *lineCalculationData;
    DerivedPlantData<data_type> derivedData;
    EulerIntegrator<LineVgEstimator<data_type>,data_type> integrator;
    LineVgEstimator()
    {
        parent::Vg_hat = typename parent::state_type(1);
        parent::Vg_hat[0] = 0;
        integrator.UL = VFParameters<data_type>::Vg_hat_est_max;
        integrator.LL = -VFParameters<data_type>::Vg_hat_est_max;
    }

    template <typename T>
    LineVgEstimator(const T& Ts):LineVgEstimator()
    {
        parent::Ts = (data_type)Ts;
    }


    template <typename T, typename U>
    LineVgEstimator(const T& Ts, const U& Vg_hat_init):LineVgEstimator(Ts)
    {
        parent::Vg_hat[0] = (data_type)Vg_hat_init;
    }
    ~LineVgEstimator(){}

    virtual data_type GetVgHat(
        PathCalculationData<data_type> *pathCalculationData,
        const DerivedPlantData<data_type>& derivedData,
        const Point<data_type> location, 
        const Vector3<data_type>& Vb
        ) override
    {
        
        data_type result = parent::Vg_hat[0];
        //do integration
        //std::cout<<"Vg_hat[0]:"<<Vg_hat[0]<<std::endl;
        this->lineCalculationData = reinterpret_cast<LineCalculationData<data_type> *>(pathCalculationData);

        Line<data_type> *line = lineCalculationData->line;
        
        this->derivedData = derivedData;
        integrator.integrate(*this,parent::Ts);
        
        return result;
    }

    virtual void operator()(state_type &y , state_type &yp , data_type t )  const override
    {

        //std::cout<<"y[0]:"<<y[0]<<std::endl;
        yp[0]=get_vg_hat_dot_line(derivedData,y[0]);
        //std::cout<<"yp[0]:"<<yp[0]<<std::endl;
        
    }

    

    data_type get_vg_hat_dot_line(
        const DerivedPlantData<data_type>& derivedData, 
        const data_type& Vg_hat) const
    {
        /*Vg_hat_dot calculation*/
        //rho = y_int^2/pi^2;   % Positive scale factor used to weight the terms so that y and chi are similar in magnitude
        data_type rho = (sq(LineVFParameters<data_type>::line_epy_init)/M_PI_SQR);

        //diff_chi_chid = chi-chi_d; % Error of course angle
        data_type diff_chi_chid = derivedData.chi - lineCalculationData->chi_d;


        //term =diff_chi_chid/epsi;
        data_type term = diff_chi_chid/LineVFParameters<data_type>::line_epsi;

        /*if abs(term)<=1
            sat=term;
        else
            sat=sign(term);
        end*/
        data_type sat_term = term;
        if(fabs(term)>1)
        {
            sat_term = sign(term);
        }
        

        //% % derivative of Vg' over x'
        data_type diff_windWs_chi = wind_phiw-derivedData.chi;
        data_type sin_of_diff_windWs_chi = sin(diff_windWs_chi);


        //der = W*sin(phiw-chi)+(Va^2-W^2*(sin(phiw-chi))^2)^(-0.5)*W^2*sin(phiw-chi)*cos(phiw-chi); % partial derivative of Vg' over chi'
        data_type der = wind_Ws*sin_of_diff_windWs_chi*(1+1/sqrt(sq(derivedData.Va)-sq(wind_Ws)*sq(sin_of_diff_windWs_chi))*wind_Ws*cos(diff_windWs_chi)); //I HAVE A DOUBT HERE : why Ws^2 in second term in matlab model?
        //std::cout << "der:" << der << std::endl;
        data_type two_over_M_PI_beta_s_sin_diff_chi_chiq = TWO_OVER_M_PI*LineVFParameters<data_type>::line_k/(1+sq(LineVFParameters<data_type>::line_k*lineCalculationData->e_y))*sin(derivedData.chi-lineCalculationData->chi_q);
        //term2 = (-1*chi_inf*2/pi*(k*sin(chi - chi_q))/(1+(k*(e))^2)*Vg_hat);
        data_type term2 = -LineVFParameters<data_type>::chi_inf*two_over_M_PI_beta_s_sin_diff_chi_chiq*Vg_hat;
        //last_term = -1*kk*sat;
        data_type last_term = -LineVFParameters<data_type>::line_kk*sat_term;


        /*Vg_hat_dot = Gamma/alpha*rho*diff_chi*chi_inf*2/pi*(k*(sin(chi-chi_q)))/(1+(k*(e))^2)...
    - sigma* Gamma * Vg_hat +...
     1*der*(term2/alpha +last_term);*/


        data_type Vg_hat_dot = LineVFParameters<data_type>::line_gamma/LineVFParameters<data_type>::est_alpha*rho*diff_chi_chid*LineVFParameters<data_type>::chi_inf*two_over_M_PI_beta_s_sin_diff_chi_chiq
        -LineVFParameters<data_type>::line_sigma*LineVFParameters<data_type>::line_gamma*Vg_hat + der*(term2/LineVFParameters<data_type>::est_alpha+last_term);
        

        return Vg_hat_dot;


    }
    
};

template<typename data_type>
class OrbitVgEstimator : public VgEstimator<data_type>
{
    public:
    typedef VgEstimator<data_type> parent;
    typedef typename parent::state_type state_type;
    EulerIntegrator<OrbitVgEstimator<data_type>,data_type> integrator;
    OrbitCalculationData<data_type> *orbitCalculationData;


    Vector3<data_type> Vb;
    Vector3<data_type> wind_velocity;
    DerivedPlantData<data_type> derivedData;
    WindModel<data_type,false> windModel;

    OrbitVgEstimator()
    {
        parent::Vg_hat = typename parent::state_type(1);
        parent::Vg_hat[0] = 0;
        integrator.UL = VFParameters<data_type>::Vg_hat_est_max;
        integrator.LL = -VFParameters<data_type>::Vg_hat_est_max;
    }

    template <typename T>
    OrbitVgEstimator(const T& Ts):OrbitVgEstimator()
    {
        parent::Ts = (data_type)Ts;
    }


    template <typename T, typename U>
    OrbitVgEstimator(const T& Ts, const U& Vg_hat_init):OrbitVgEstimator(Ts)
    {
        parent::Vg_hat[0] = (data_type)Vg_hat_init;
    }
    ~OrbitVgEstimator(){}

    virtual data_type GetVgHat(
        PathCalculationData<data_type> *pathCalculationData,
        const DerivedPlantData<data_type> &derivedData,
        const Point<data_type> location,
        const Vector3<data_type> &Vb) override
    {
        data_type result = parent::Vg_hat[0];
        //do integration
        this->wind_velocity = windModel.GetTimeVaryingComponent(parent::t);
        this->Vb = Vb;
        this->orbitCalculationData = reinterpret_cast<OrbitCalculationData<data_type> *>(pathCalculationData);
        Orbit<data_type> *orbit=  orbitCalculationData->orbit;

        // auto deltaVector = location - orbit->centre;
        // data_type gamma = atan2(deltaVector.y,deltaVector.x);
        // data_type d = deltaVector.norm();
        // MatchDomain(gamma,derivedData.chi);
        // data_type d_tilde = d - orbit->radius;

        // data_type chi_d = gamma + orbit->lambda*(M_PI/2 + atan(OrbitVFParameters<data_type>::k*(d_tilde)));



        this->derivedData = derivedData;
        integrator.integrate(*this,parent::Ts);

        //GetVgHat(mission,derivedData,location,chi_d,e_y,gamma,d);
        
        return result;
    }

    virtual void operator()(
        state_type &y , 
        state_type &yp , 
        data_type t )  const override
    {

        yp[0]=get_vg_hat_dot_orbit(derivedData,y[0]);
        
    }



    inline data_type get_vg_hat_dot_orbit(
        const DerivedPlantData<data_type>& derivedData,    
        const data_type& Vg_hat
        ) const
    {
        /*%% Parameters
        W = param(1);
        phiw = param(2);
        Va  = param(3);
        k  = param(4);
        kk = param(5);
        epsi = param(6);
        d_int = param(7);
        R  = param(8);
        j = param(9);
        Gamma = param(10);
        sigma = param(11);*/

        //%% Estimator of Vg'

        //err_1 = (phiw - chi);
        data_type err_1 = wind_phiw - derivedData.chi;
        
        //err_2 = chi - gamma;
        data_type err_2 = derivedData.chi - orbitCalculationData->gamma;
        //std::cout<<"err_2:"<<err_2<<std::endl;

        //chi_tilde = chi - chi_d;
        data_type chi_tilde = derivedData.chi - orbitCalculationData->chi_d;
        
        //term = chi_tilde / epsi;
        data_type term = chi_tilde/OrbitVFParameters<data_type>::epsi;
        


        //rho = (d_int / pi)^2;
        data_type rho = sq(OrbitVFParameters<data_type>::d_init)/M_PI_SQR;
        
        /*if abs(term) <= 1
            chi_tilde_eps = term;
        else
            chi_tilde_eps = sign(term);
        end*/
        data_type chi_tilde_eps = term;
        if(fabs(term)>1)
        	chi_tilde_eps = sign(term);
        
        //WIND
         Vector3<data_type> Vg_vec = Vb - wind_velocity;

         data_type Vg = Vg_vec.norm();

        //data_type Vg = Vg_hat;

        /*if Vg_hat - Vg > 0.5 ||  Vg_hat - Vg < -0.5
            sigma = 3*(Vg_hat - Vg);
        else
            sigma = 0;
        end */

        data_type sigma = 0;
        if (fabs(Vg_hat - Vg)>0.5)
            sigma = 3*(Vg_hat - Vg);

        
        //% Partial derivative of Vg' over chi'
        //beta = k/(1 + (k * (d - R))^2);
        data_type beta = OrbitVFParameters<data_type>::k /(1 + sq(OrbitVFParameters<data_type>::k * (orbitCalculationData->d - orbitCalculationData->orbit->radius)));

        
        //der = W * sin(err_1) + (Va^2 - W^2 * (sin(err_1))^2)^(-0.5) * W^2 * sin(err_1) * cos(err_1); 
        data_type der = wind_Ws * sin(err_1) + 1/sqrt(sq(derivedData.Va) - sq(wind_Ws) * sq(sin(err_1))) * sq(wind_Ws) * sin(err_1) * cos(err_1); 
        
        //term1 = -Gamma * rho * (chi_tilde) * (sin(err_2) / d + j * beta * cos(err_2));

        data_type term1 = -OrbitVFParameters<data_type>::Gamma * rho * (chi_tilde) * (sin(err_2) / orbitCalculationData->d + orbitCalculationData->orbit->direction * beta * cos(err_2));
        //std::cout<<"term1:"<<term1<<std::endl;
        //term2 = der*(Vg_hat*sin(err_2)/d + j*beta*Vg_hat*cos(err_2) - kk*chi_tilde_eps);
        data_type term2 = der*(Vg_hat*sin(err_2)/orbitCalculationData->d + orbitCalculationData->orbit->direction*beta*Vg_hat*cos(err_2) - OrbitVFParameters<data_type>::kk*chi_tilde_eps);
        
        //data_type Vg_hat_dot = 0;
        /*Vg2_hat_dot =  term1 + 1*term2 - sigma*Gamma*Vg_hat;*/
        data_type Vg_hat_dot = term1 + term2 - sigma*OrbitVFParameters<data_type>::Gamma*Vg_hat;


        return Vg_hat_dot;


    }
};

#endif