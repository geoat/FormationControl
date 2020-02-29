


#ifndef DRONEMODEL_HPP
#define DRONEMODEL_HPP
#include <typeinfo> 
#include <exception>
#include <utility>
#include <memory>
#include <cmath> //for abs
#include <iterator>

#include "uav_constants.hpp"
#include "../Maths/math_helper.hpp"
#include "lookuptable.hpp"
#include "../Maths/phase_unwrapper.hpp"
#include "../Maths/windup.hpp"
#include "../Maths/pid_helper.hpp"
#include "derivedplantdata.hpp"

  //Helper Functions Imlementation
  //Function to wrap angles




//For using Eigen Library for matrix calculations
#include <Eigen/Dense>

//For using odeint library
#include <boost/numeric/odeint.hpp>

// defines to extract phi,theta,psi from a euler angle vector.
#define phi(x) (x)(0)
#define theta(x) (x)(1)
#define psi(x) (x)(2)



using namespace boost::numeric::odeint;

//Template generlaiztion to handle invalide use of the class
template<typename data_type, bool lowlevelControllers, bool controlAllocator>
class Drone;

//only 6DOF - no control allocator- no lowlevel controllers 
template<typename data_type>
class Drone<data_type,false,false>
{
    public:

        //forward declaration of helper class to calculate derived data from given states
        //class DerivedPlantData;

        //declaring matrix and vector types used
        typedef Eigen::Matrix<data_type, 3, 1> vector_type;
        typedef Eigen::Matrix<data_type, 3, 3> matrix_type;

        //declaring the statet type and variable
        typedef std::vector<data_type> state_type;
        state_type y;

        state_type acceleration;
        state_type wbdot;

        //defining types for odeint integration
        typedef runge_kutta_dopri5<state_type> dopri_stepper_type;
        typedef runge_kutta_cash_karp54<state_type> cashkarp54_stepper_type;
        //typename make_dense_output<dopri_stepper_type>::type stepper;
        typedef controlled_runge_kutta<runge_kutta_dopri5<state_type>> stepper_type;
        dense_output_runge_kutta<stepper_type> stepper;
        //error tolerance
        data_type relerr = 1E-5;
        data_type abserr = 1E-6;

    public:
    //drone paramters
    data_type Ts = 0.0025;
    
    data_type Fxyz[3];
    data_type Mxyz[3];

    data_type Mass;
    data_type It[3][3];
    data_type IInvt[3][3];
    data_type t = 0;



    //wind_vector
    data_type wind_vector[3] = {0,0,0};

    //default constructor
    Drone(){}
    //default destrcuctor 
    ~Drone(){}
    
    //constructor for intializing all required parameters
    Drone
    (
        data_type& Ts,
        data_type& Mass, 
        const vector_type& Xe_init, 
        const vector_type& Vb_init, 
        const vector_type& Euler_init, 
        const vector_type& pqr_init, 
        int numberofStates =12
        ):Ts(Ts),Mass(Mass)
    {
        //std::cout<<"Mass:"<<Mass<<std::endl;
        //defining the number of states required
        y = state_type(numberofStates,0);
        acceleration = state_type(3,0);
        wbdot = state_type(3,0);
        //Setting parameters
        FillIArray(It,Constants_Bxl_I);
        FillIArray(IInvt,Constants_Bxl_IInv);
        y[0] = Xe_init(0);
        y[1] = Xe_init(1);
        y[2] = Xe_init(2);

        y[3] = Vb_init(0);
        y[4] = Vb_init(1);
        y[5] = Vb_init(2);

        y[6] = Euler_init(0);
        y[7] = Euler_init(1);
        y[8] = Euler_init(2);

        y[9] = pqr_init(0);
        y[10] = pqr_init(1);
        y[11] = pqr_init(2);

        get_acc(y,acceleration.begin(),{0,0,0},this->Mass);
        get_wbdot(y,wbdot.begin(),{0,0,0});

        // std::cout<<"DroneConfiguration"<<std::endl;
        // std::cout<<"Ts:"<<Ts<<std::endl;
        // std::cout<<"Mass:"<<Mass<<std::endl;
        // for(size_t i = 0; i < 3; i++)
        // {
        //     for(size_t j = 0; j < 3; j++)
        //     {
        //         std::cout<<"I["<<i<<"]["<<j<<"]:"<<It[i][j]<<",";
        //     }
            
        // }
        // std::cout<<std::endl;
        // std::cout<<"XeInit:"<<Xe_init<<std::endl;
        // std::cout<<"Vb_init:"<<Vb_init<<std::endl;        
        // std::cout<<"Euler_init:"<<Euler_init<<std::endl;             
        // std::cout<<"pqr_init:"<<pqr_init<<std::endl;   
    }

    Drone
    (
        data_type& Ts,
        data_type& Mass, 
        matrix_type& I,
        const vector_type& Xe_init, 
        const vector_type& Vb_init, 
        const vector_type& Euler_init, 
        const vector_type& pqr_init, 
        int numberofStates =12
        ):Drone(Ts,Mass,Xe_init,Vb_init,Euler_init,pqr_init,numberofStates)
    {
        FillIArray(It,I);
        FillIArray(IInvt,I.inverse());
    }



    void FillIArray(data_type (&I)[3][3],std::initializer_list<std::initializer_list<data_type>> listoflist)
    {
        int rowLength = listoflist.size();
        int columnLength = listoflist.begin()->size();
        //std::cout<<"mxn="<<rowLength<<"x"<<columnLength<<std::endl;

        for(auto i = listoflist.begin(); i != listoflist.end(); i++)
        {
            if(i->size()!=columnLength) throw "column length missmatch";
            int pos = std::distance(listoflist.begin(),i);
            std::uninitialized_copy(i->begin(),i->end(), I[pos]);
        }
    }

    void FillIArray
    (
        data_type (&Iarray)[3][3],
        const matrix_type& I
        )
    {
        for(size_t i = 0; i < 3; i++)
        {
            for(size_t j = 0; j < 3; j++)
            {
                Iarray[i][j] = I(i,j);
            }
        }
        
    }
    
    virtual void operator()(state_type &y , state_type &yp , data_type t )  const
    {
        //DerivedPlantData temp(y,wind_vector);

            
        //DerivedPlantData temp = get_derived_data(y,wind_vector);
        //data_type F[3] = {Fxyz(0),Fxyz(1), Fxyz(2)};
        //data_type Mo[3] = {Mxyz(0),Mxyz(1), Mxyz(2)};
        get_y_dot(y,yp,t,DerivedPlantData<data_type>(y,wind_vector),Fxyz,Mxyz);
        
    }


    //template<class C, typename T>
    inline void get_acc(state_type& y, typename state_type::iterator acc, const data_type (&Fxyza)[3], const data_type& Mass) const
    {

        //Ab = (Vb.cross(Wb) + Fxyz/Mass);        
        *(acc) = y[4]*y[11] - y[5]*y[10] + Fxyza[0]/Mass;
        *(acc + 1) = y[5]*y[9] - y[3]*y[11] + Fxyza[1]/Mass;
        *(acc + 2) = y[3]*y[10] - y[4]*y[9] + Fxyza[2]/Mass;
    }


    inline void get_wbdot(state_type& y, typename state_type::iterator wbdot, const data_type (&Mxyza)[3]) const
    {

//Wb_dot = IInv*(Mxyz-Wb.cross(I*Wb));
        
        data_type temp[3];
        data_type temp2[3];

        temp[0] = It[0][0]*y[9] + It[0][1]*y[10] + It[0][2]*y[11];
        temp[1] = It[1][0]*y[9] + It[1][1]*y[10] + It[1][2]*y[11];
        temp[2] = It[2][0]*y[9] + It[2][1]*y[10] + It[2][2]*y[11];

        temp2[0] = y[11]*temp[1] - y[10]*temp[2];
        temp2[1] = y[9]*temp[2] - y[11]*temp[0];
        temp2[2] = y[10]*temp[0] - y[9]*temp[1];
        


        temp[0] = Mxyza[0] + temp2[0];
        temp[1] = Mxyza[1] + temp2[1];
        temp[2] = Mxyza[2] + temp2[2];

    
        *wbdot  = IInvt[0][0]*temp[0] + IInvt[0][1]*temp[1] + IInvt[0][2]*temp[2];
        *(wbdot + 1) = IInvt[1][0]*temp[0] + IInvt[1][1]*temp[1] + IInvt[1][2]*temp[2];
        *(wbdot + 2) = IInvt[2][0]*temp[0] + IInvt[2][1]*temp[1] + IInvt[2][2]*temp[2];
    }

    inline void get_y_dot (state_type& y, state_type& yp, double t,const DerivedPlantData<data_type>& derivedData, const data_type (&Fxyza)[3],const data_type (&Mxyza)[3] ) const
    {

        
        //Switching namespaces todo matrixcalculations

        //creating maps between y (state vector) and corresponding matrices
        //Map<vector_type> Xe(&y[0]);
        //Map<vector_type> Vb(&y[3]);
        //Map<vector_type> euler(&y[6]);
        //Map<vector_type> Wb(&y[9]);
        //creating maps between yp (state deriative) and corresponding matrices
        //Map<vector_type> Ve(&yp[0]);
        //Map<vector_type> Ab(&yp[3]);
        //Map<vector_type> Euler_dot(&yp[6]);
        //Map<vector_type> Wb_dot(&yp[9]);

    
        
        // matrix_type R_B_E;
        //     R_B_E<<cos_theta*cos_psi, cos_theta*sin_psi, -sin_theta,
        //         -cos_phi*sin_psi+sin_phi*sin_theta*cos_psi, cos_phi*cos_psi+sin_phi*sin_theta*sin_psi, sin_phi*cos_theta,
        //         sin_phi*sin_psi+cos_phi*sin_theta*cos_psi, -sin_phi*cos_psi+cos_phi*sin_theta*sin_psi, cos_phi*cos_theta;


      //matrix_type R_E_B;
            /*R_E_B<<cos_theta*cos_psi,-cos_phi*sin_psi+sin_phi*sin_theta*cos_psi,sin_phi*sin_psi+cos_phi*sin_theta*cos_psi,
                cos_theta*sin_psi, cos_phi*cos_psi+sin_phi*sin_theta*sin_psi, -sin_phi*cos_psi+cos_phi*sin_theta*sin_psi,
                -sin_theta, sin_phi*cos_theta, cos_phi*cos_theta;*/


        //Ve = R_E_B*Vb;
        yp[0] = derivedData.Ve[0];
        yp[1] = derivedData.Ve[1];
        yp[2] = derivedData.Ve[2];

        //Ab = (Vb.cross(Wb) + Fxyz/Mass);        
        //yp[3] = y[4]*y[11] - y[5]*y[10] + Fxyza[0]/Mass;
        //yp[4] = y[5]*y[9] - y[3]*y[11] + Fxyza[1]/Mass;
        //yp[5] = y[3]*y[10] - y[4]*y[9] + Fxyza[2]/Mass;
        get_acc(y,(yp.begin()+3),Fxyza,Mass);


        /* matrix_type J;
            J<<1,derivedData.sin_phi*derivedData.tan_theta,derivedData.cos_phi*derivedData.tan_theta,
            0,derivedData.cos_phi,-derivedData.sin_phi,
            0,derivedData.sin_phi/derivedData.cos_theta,derivedData.cos_phi/derivedData.cos_theta;
 
        //Euler_dot = J*Wb;*/
 
        yp[6] = y[9] + y[11]*derivedData.cos_phi*derivedData.tan_theta + y[10]*derivedData.sin_phi*derivedData.tan_theta;
        yp[7] =  y[10]*derivedData.cos_phi - y[11]*derivedData.sin_phi;
        yp[8] = (y[11]*derivedData.cos_phi)/derivedData.cos_theta + (y[10]*derivedData.sin_phi)/derivedData.cos_theta;
        

        get_wbdot(y,(yp.begin()+9),Mxyza);
    }

    inline void CalculateAccelerations()
    {
        get_acc(y,acceleration.begin(),Fxyz,this->Mass);
        get_wbdot(y,wbdot.begin(),Mxyz);

    }
    
    virtual void DoNextStep(const data_type (&Fxyzi)[3],const data_type (&Mxyzi)[3])
    {
        Fxyz[0] = Fxyzi[0];
        Fxyz[1] = Fxyzi[1];
        Fxyz[2] = Fxyzi[2];
        Mxyz[0] = Mxyzi[0];
        Mxyz[1] = Mxyzi[1];
        Mxyz[2] = Mxyzi[2];


        CalculateAccelerations();
        integrate(*this);

        
    }
    template<class System>
    inline void integrate(System& system)
    {   
        //typedef controlled_runge_kutta< runge_kutta_cash_karp54< state_type > > stepper_typetest;
        //boost::numeric::odeint::integrate_adaptive(stepper_typetest(),system, y, t, t+Ts, Ts);
        boost::numeric::odeint::integrate_adaptive(make_dense_output<dopri_stepper_type>( abserr, relerr ),system, y, t, t+Ts, Ts);
        //boost::numeric::odeint::integrate_const(make_dense_output<dopri_stepper_type>( abserr, relerr ),system, y, t, t+Ts, Ts);
        //boost::numeric::odeint::integrate_const(runge_kutta4< state_type >(),system, y, t, t+Ts, Ts);

        wrapToPi(this->y.data()+6,3);


        t+=Ts;
    }

    DerivedPlantData<data_type> get_derived_data(const state_type &y, const data_type (&wind_vector)[3]) const
    {
        return DerivedPlantData<data_type>(y,wind_vector);
    }

    void wrapToPi(data_type *p, int n)
    {

        for (int i = 0; i < n; i++)
        {
            while (p[i] > M_PI)
            {
                p[i] = p[i] - 2 * M_PI;
            }
            while (p[i] < -M_PI)
            {
                p[i] = p[i] + 2 * M_PI;
            }
        }
    }
};

//6DOF with controlAllocator
template<typename data_type>
class Drone<data_type,false,true>: public Drone<data_type,false,false>
{
    public:

        typedef Drone<data_type, false, false> BareDrone;
        typedef typename BareDrone::vector_type vector_type;
        typedef typename BareDrone::matrix_type matrix_type;
        typedef typename BareDrone::state_type state_type;
        typedef double lookup_type;

        data_type throttle;
        data_type delta_ail;
        data_type delta_elev;
        data_type delta_rudd;

        //lookup tables
        Elements_Array<lookup_type> alpha_elements;
        Elements_Array<lookup_type> ail_elements;
        OneDLookUpTable<lookup_type, lookup_type, 9> alpha_lookup;
        TwoDLookUpTable<lookup_type, lookup_type> alpha_ail_lookup;
        OneDLookUpTable<lookup_type, lookup_type, 1> ail_lookup;
        TwoDLookUpTable<lookup_type, lookup_type> alpha_elev_lookup;
        //OneDLookUpTable<lookup_type,lookup_type,9> alpha_lookup(alpha_elements,Elements_Array<lookup_type>(Constants_Aero_CD_Basic),Elements_Array<lookup_type>(Constants_Aero_Cy_RollRate),Elements_Array<lookup_type>(Constants_Aero_CL_Basic),Elements_Array<lookup_type>(Constants_Aero_Cl_Beta),Elements_Array<lookup_type>(Constants_Aero_Cl_RollRate),Elements_Array<lookup_type>(Constants_Aero_Cl_YawRate),Elements_Array<lookup_type>(Constants_Aero_Cm_Basic),Elements_Array<lookup_type>(Constants_Aero_Cn_RollRate),Elements_Array<lookup_type>(Constants_Aero_Cn_YawRate));

        Drone() {}

        Drone(
            data_type &Ts,
            data_type &Mass,
            const vector_type &Xe_init,
            const vector_type &Vb_init,
            const vector_type &Euler_init,
            const vector_type &pqr_init,
            int numberofStates = 12) : BareDrone(Ts, Mass, Xe_init, Vb_init, Euler_init, pqr_init, numberofStates)
        {

            alpha_elements = Elements_Array<lookup_type>(Constants_Aero_alpha_vector);
            alpha_lookup = OneDLookUpTable<lookup_type, lookup_type, 9>(alpha_elements, Elements_Array<lookup_type>(Constants_Aero_CD_Basic), Elements_Array<lookup_type>(Constants_Aero_Cy_RollRate), Elements_Array<lookup_type>(Constants_Aero_CL_Basic), Elements_Array<lookup_type>(Constants_Aero_Cl_Beta), Elements_Array<lookup_type>(Constants_Aero_Cl_RollRate), Elements_Array<lookup_type>(Constants_Aero_Cl_YawRate), Elements_Array<lookup_type>(Constants_Aero_Cm_Basic), Elements_Array<lookup_type>(Constants_Aero_Cn_RollRate), Elements_Array<lookup_type>(Constants_Aero_Cn_YawRate));
            ;
            ail_elements = Elements_Array<lookup_type>(Constants_Aero_delta_ail_vector);
            alpha_ail_lookup = TwoDLookUpTable<lookup_type, lookup_type>(alpha_elements, ail_elements, TwoDOutputValues<lookup_type>(Constants_Aero_Cn_ail));
            ail_lookup = OneDLookUpTable<lookup_type, lookup_type, 1>(ail_elements, Elements_Array<lookup_type>(Constants_Aero_Cl_ail));
            //!Wrong
            alpha_elev_lookup = TwoDLookUpTable<lookup_type, lookup_type>(alpha_elements, ail_elements, TwoDOutputValues<lookup_type>(Constants_Aero_Aero_CD_elev));
    }

    Drone
    (
        data_type& Ts,
        data_type& Mass,
        matrix_type& I,
        const vector_type& Xe_init, 
        const vector_type& Vb_init, 
        const vector_type& Euler_init, 
        const vector_type& pqr_init,
        int numberofStates =12
        ):Drone(Ts,Mass,Xe_init,Vb_init,Euler_init,pqr_init,numberofStates)
    {
        BareDrone::FillIArray(BareDrone::It,I);
        BareDrone::FillIArray(BareDrone::IInvt,I.inverse());
    }

    virtual void operator()(state_type &y , state_type &yp , data_type t )  const override
    {
        
        get_y_dot(y,yp,t,DerivedPlantData<data_type>(y,BareDrone::wind_vector),throttle,delta_ail,delta_elev,delta_rudd);
        
    }

    inline void get_y_dot (
        state_type& y, 
        state_type& yp, 
        double t,
        const DerivedPlantData<data_type>& derivedData,
        const data_type& throttle,
        const data_type& delta_ail,
        const data_type& delta_elev,
        const data_type& delta_rudd
        ) const
    {
        data_type Fxyz[3];
        data_type Mxyz[3];
        calculateFandM(derivedData,throttle,delta_ail,delta_elev,delta_rudd,Fxyz,Mxyz);
        BareDrone::get_y_dot(y,yp,t,derivedData,Fxyz,Mxyz);
        
    }
    
    virtual void DoNextStep(
        const data_type& throttlei,
        const data_type& delta_aili,
        const data_type& delta_elevi,
        const data_type& delta_ruddi
    ) 
    {
     /*    static int count = 0;
        count++;
        std::cout<<"Called"<<count<<std::endl; */
           throttle = throttlei;
           delta_ail = delta_aili;
           delta_elev = delta_elevi;
           delta_rudd = delta_ruddi;


        
        calculateFandM(DerivedPlantData<data_type>(BareDrone::y,BareDrone::wind_vector),throttle,delta_ail,delta_elev,delta_rudd,BareDrone::Fxyz,BareDrone::Mxyz);   

        BareDrone::CalculateAccelerations();
        BareDrone::integrate(*this);

        //boost::numeric::odeint::integrate_const(make_dense_output<dopri_stepper_type>( BareDrone::abserr, BareDrone::relerr ),*this, y, t, t+Ts, Ts);
        
        //stepper.do_step(*this);
        //t+=Ts;
        
    } 

    inline void calculateFandM(
        const DerivedPlantData<data_type>& derivedData,
        const data_type& throttle,
        const data_type& delta_ail,
        const data_type& delta_elev,
        const data_type& delta_rudd,
        data_type (&Fxyz)[3],
        data_type (&Mxyz)[3]
    ) const
    {
        #define FandMConst1 0.5 * Constants_C_rho * Constants_Bxl_S_prop * Constants_Bxl_rotor_thrust_coeff

        data_type Thrust= (data_type) FandMConst1*(sq((data_type)Constants_Bxl_r_prop*0.5*(Constants_Bxl_k_motor * (throttle) + Constants_Bxl_q_motor)) - sq(derivedData.Va));
        data_type FT[3] = {Thrust*Constants_CosMountAngle,0,Thrust*Constants_SinMountAngle};

        data_type MT[3] = {0,Thrust*-Constants_Bxl_propDist,0};

        data_type alpha_deg = derivedData.alpha*180/M_PI;

        lookup_type Coeff_Values_aplha[9];
        alpha_lookup.interpolate(Coeff_Values_aplha,alpha_deg);


        data_type CD_aero = Coeff_Values_aplha[0];
        data_type Cy_aero = Coeff_Values_aplha[1]*Constants_Bxl_b*BareDrone::y[9]/2/derivedData.Va+Constants_Aero_Cy_Beta*derivedData.beta;

        
        data_type CL_aero = Coeff_Values_aplha[2]+Constants_Aero_CL_PitchRate*Constants_Bxl_c*BareDrone::y[10]/2/derivedData.Va;

        data_type Cl_aero = Coeff_Values_aplha[3]*derivedData.beta+Coeff_Values_aplha[4]*Constants_Bxl_b*BareDrone::y[9]/2/derivedData.Va+Coeff_Values_aplha[5]*Constants_Bxl_b*BareDrone::y[11]/2/derivedData.Va;
 
        data_type Cm_aero = Coeff_Values_aplha[6]+Constants_Aero_Cm_PitchRate*Constants_Bxl_c*BareDrone::y[10]/2/derivedData.Va;


        data_type Cn_aero = Coeff_Values_aplha[7]*Constants_Bxl_b*BareDrone::y[9]/2/derivedData.Va+Coeff_Values_aplha[8]*Constants_Bxl_b*BareDrone::y[11]/2/derivedData.Va + Constants_Aero_Cn_Beta*derivedData.beta;


        //Can be Improved little
        data_type Cn_ail = alpha_ail_lookup.interpolate(alpha_deg,delta_ail);

        data_type Cl_ail = ail_lookup.interpolate(delta_ail,0);

        data_type CD_elev = alpha_elev_lookup.interpolate(alpha_deg,delta_elev);


        data_type CL_elev = Constants_Aero_CL_elev*delta_elev;
        data_type Cm_elev = Constants_Aero_Cm_elev*delta_elev;


        data_type Cy_rudd = -Constants_Aero_Cy_rudd * delta_rudd;
        data_type Cl_rudd = -Constants_Aero_Cl_rudd * delta_rudd;
        data_type Cn_rudd = Constants_Aero_Cn_rudd * delta_rudd;



        data_type CD = CD_aero + CD_elev;
        data_type Cy = Cy_aero+Cy_rudd;
        data_type CL = CL_aero + CL_elev;
        data_type Cl = Cl_aero + Cl_ail+ Cl_rudd;
        data_type Cm = Cm_aero + Cm_elev;
        data_type Cn = Cn_aero + Cn_ail + Cn_rudd;


        // z_8 = Variables.z8;
        // z_9 = 0;
        // DCM_bw = [cos(z_8)*cos(z_9), -cos(z_8)*sin(z_9), -sin(z_8);
        //         sin(z_9), cos(z_9), 0 ;
        //         sin(z_8)*cos(z_9), -sin(z_8)*sin(z_9), cos(z_8);];

        // z_8 = Variables.z8;
        data_type s_alpha = sin(derivedData.alpha);
        data_type c_alpha = cos(derivedData.alpha);

        data_type Cx = CL*s_alpha - CD*c_alpha;

        data_type Cz = - CL*c_alpha - CD*s_alpha;

        data_type temp = derivedData.q_bar*Constants_Bxl_S;
        data_type FA[3]={Cx*temp,Cy*temp,Cz*temp};


        data_type MA[3] = { Cl*Constants_Bxl_c*temp, Cm*Constants_Bxl_b*temp, Cn*Constants_Bxl_c*temp};
        data_type GF = Constants_g*BareDrone::Mass;

        data_type FG[3] = {-GF*derivedData.sin_theta, GF*derivedData.cos_theta*derivedData.sin_phi, GF*derivedData.cos_phi*derivedData.cos_theta};
  
        Fxyz[0]=FG[0]+FT[0]+FA[0];
        Fxyz[1]=FG[1]+FT[1]+FA[1];
        Fxyz[2]=FG[2]+FT[2]+FA[2];



        Mxyz[0]=MT[0]+MA[0];
        Mxyz[1]=MT[1]+MA[1];
        Mxyz[2]=MT[2]+MA[2];


    }

    

};


template<typename data_type>
class Drone<data_type,true,true>: public Drone<data_type,false,true>
{
    private:
    
    windup<data_type> windup_with_Ail_Dem = windup<data_type>(45,-45);
    windup<data_type> windup_with_Throttle_Dem = windup<data_type>(100,0);
    windup<data_type> windup_with_Pitch_Dem = windup<data_type>(30,-30);
    windup<data_type> windup_with_Elev_Dem = windup<data_type>(45,-45);
    
    pid<data_type> pid_Ail_Dem = pid<data_type>(Constants_PID2_P,Constants_PID2_I,Constants_PID2_D, Constants_PID2_FF, Constants_PID2_I_Sat);
    pid<data_type> pid_Elev_Dem = pid<data_type>(Constants_PID7_P,Constants_PID7_I,Constants_PID7_D, Constants_PID7_FF, Constants_PID7_I_Sat);

    
    public:

        class DerivedPlantData_Adv;
        typedef Drone<data_type, false, false> BareDrone;
        typedef Drone<data_type, false, true> DroneWithCA;
        typedef typename DroneWithCA::BareDrone::vector_type vector_type;
        typedef typename DroneWithCA::BareDrone::matrix_type matrix_type;
        typedef typename DroneWithCA::BareDrone::state_type state_type;
        data_type Ab_y = 0;

        data_type chi_c = 0;
        data_type VT = 0;
        data_type HT = 0;

        phase_unwrapper<double, true> phase_unwrap_chi;
        phase_unwrapper<double, true> phase_unwrap_chi_c;
        Drone() {}

        Drone(
            data_type &Ts,
            data_type &Mass,
            const vector_type &Xe_init,
            const vector_type &Vb_init,
            const vector_type &Euler_init,
            const vector_type &pqr_init,
            int numberofStates = 25) : DroneWithCA(Ts, Mass, Xe_init, Vb_init, Euler_init, pqr_init, numberofStates)
        {      
     
    }

    Drone
    (
        data_type& Ts,
        data_type& Mass,
        matrix_type& I,
        const vector_type& Xe_init, 
        const vector_type& Vb_init, 
        const vector_type& Euler_init, 
        const vector_type& pqr_init,
        int numberofStates =25
        ):Drone(Ts,Mass,Xe_init,Vb_init,Euler_init,pqr_init,numberofStates)
    {      
        BareDrone::FillIArray(BareDrone::It,I);
        BareDrone::FillIArray(BareDrone::IInvt,I.inverse());
    }


    virtual void operator()(state_type &y , state_type &yp , data_type t )  const override
    {
        
        get_y_dot(y,yp,t,DerivedPlantData_Adv(y,BareDrone::wind_vector),chi_c,VT,HT);
        
    }

    inline void get_y_dot (
        state_type& y, 
        state_type& yp, 
        double t,
        const DerivedPlantData_Adv& derivedData,
        const data_type& chi_c,
        const data_type& VT,
        const data_type& HT
        ) const
    {
        data_type VTSq = VT*VT;    

        data_type chi_deg = derivedData.chi*180/M_PI;        
        chi_deg = phase_unwrap_chi.unwrap_temp(chi_deg);
        
        data_type error = chi_c - chi_deg;
        // std::cout<<"derivedData.chi:"<<derivedData.chi<<std::endl;
        // std::cout<<"chi_c:"<<chi_c<<std::endl;
        // std::cout<<"chi_deg:"<<chi_deg<<std::endl;
        yp[12] = -Constants_N*y[12] + Constants_N_pow_2*(error);
        // std::cout<<"Constants_N_pow_2:"<<Constants_N_pow_2<<std::endl;
        // std::cout<<"error:"<<error<<std::endl;
        // std::cout<<"yp[12]:"<<yp[12]<<std::endl;

        
        data_type Roll_Reference = sat( Constants_P1*error+Constants_D1N*error - Constants_D1*y[12],45,-45);
        data_type tempI = derivedData.scalerSq*Constants_PID2_I*(sat(Constants_KRoll*(Roll_Reference-180/M_PI*y[6]),Constants_srate,-Constants_srate)-180/M_PI*y[9]);
        yp[13] = windup_with_Ail_Dem.get_value(tempI) + (sat(y[13],Constants_PID2_I_Sat,-Constants_PID2_I_Sat)-y[13])/BareDrone::Ts;

       
        // %% x3_dot
        data_type STE_err_temp = calc_STE_err(VTSq,HT,derivedData.VaSq,derivedData.h);
        yp[14] = windup_with_Throttle_Dem.get_value(STE_err_temp);

        // %% x4_dot
        yp[15] = -Constants_N3*y[15]+Constants_N3_pow_2*STE_err_temp;

        // %% x5_dot
        data_type SBE_err_temp = calc_SBE_err(VTSq,HT,derivedData.VaSq,derivedData.h);
        yp[16] = windup_with_Pitch_Dem.get_value(SBE_err_temp);

        // %% x6_dot
        yp[17] = -Constants_N4*y[17]+Constants_N4_pow_2*SBE_err_temp;
        
        // %% x7_dot
        data_type throttle_dem = sat(calc_Throttle_Dem_unsat(STE_err_temp,y),100,0);
        std::cout.precision(15);

        data_type pitch_dem = sat(calc_Pitch_Dem_unsat(SBE_err_temp,y),30,-30);
        
        data_type  y7_rad_sat_80 = sat(y[6],80,-80);


        
        data_type  z5_Temp_for_x7_dot = derivedData.scalerSq*Constants_PID7_I*(sat(Constants_K5*(pitch_dem-180/M_PI*y[7]),Constants_S5UP,Constants_S5DN)+Constants_C5*fabs(Constants_g*tan(y7_rad_sat_80)*sin(y7_rad_sat_80)/derivedData.Va)*cos(y[7])*180/M_PI-180/M_PI*y[10]);
        //std::cout<<"z5_Temp_for_x7_dot:"<<z5_Temp_for_x7_dot<<std::endl;
        yp[18]  = windup_with_Elev_Dem.get_value(z5_Temp_for_x7_dot) +(sat(y[18],Constants_PID7_I_Sat,-Constants_PID7_I_Sat)-y[18])/BareDrone::Ts;
  
   
        // %% x8_dot
        data_type z6_temp = Constants_g*Constants_Ctr_Yaw_YAW2SRV_RLL/std::max(derivedData.Va,(data_type)Constants_APlane_airspeed_min);
        
        data_type x8_Int_input= y[11] - z6_temp*tan(y[6])*cos(y[6]);
        yp[19] = -1.6*y[19]+1.6*(x8_Int_input);

        // %% x9_dot

        yp[20] = (Constants_K7*Ab_y-(x8_Int_input-y[19]))*Constants_K8;

        
        // %% x10_dot
        yp[21] = -45*y[21]+45*throttle_dem;
        
        // %% x11_dot
        data_type ail_dem = sat(calc_Ail_Dem_unsat(y,Roll_Reference,derivedData),45,-45);
        yp[22] = -45*y[22]+45*ail_dem;
        
        // %% x12_dot
        data_type elev_dem = -sat(calc_Elev_Dem_unsat(y,pitch_dem,derivedData),45,-45);
        yp[23] = -45*y[23]+45*elev_dem;
        
        // %% x13_dot
        data_type rudder_dem =calc_Rudder_dem(y,ail_dem,derivedData);

        yp[24] = -45*y[24]+45*rudder_dem;
        
        
        data_type throttle = sat(y[21],100,0);
        data_type delta_ail = sat(y[22],30,-30);
        data_type delta_elev= sat(y[23],15,-15);
        data_type delta_rudd = sat(y[24],25,-25);
  
        DroneWithCA::get_y_dot(y,yp,t,derivedData,throttle,delta_ail,delta_elev,delta_rudd);

            // if(BareDrone::t<1 ){
            // std::cout<<"t:"<<BareDrone::t<<std::endl;
            
            // std::cout<<"u(1):"<<chi_c<<std::endl;

            // std::cout<<"chi:"<<chi_deg<<std::endl;
            // for(size_t i = 0; i < 13; i++)
            // {
            //     std::cout<<y[12+i]<<",";
            // }
            // for(size_t i = 0; i < 12; i++)
            // {
            //     std::cout<<y[i]<<((i==11)?"\n":",");
            // }

            // for(size_t i = 0; i < 13; i++)
            // {
            //     std::cout<<yp[12+i]<<",";
            // }
            // for(size_t i = 0; i < 12; i++)
            // {
            //     std::cout<<yp[i]<<((i==11)?"\n":",");
            // }
            // }
    }

    inline data_type calc_STE_err(const data_type& VTSq, const data_type& HT, const data_type& VaSq, const data_type& h) const
    {
        return (Constants_g*HT+0.5*VTSq) - (Constants_g*h + 0.5*VaSq);
    }

    inline data_type calc_SBE_err(const data_type& VTSq, const data_type& HT, const data_type& VaSq, const data_type& h) const
    {
        return Constants_g*HT-VTSq/2 - Constants_g*h + VaSq/2;
    }
    inline data_type calc_Throttle_Dem_unsat(data_type& STE_err, state_type& y) const
    {
        return Constants_P3*(STE_err) + Constants_I3*y[14] + Constants_D3*Constants_N3*(STE_err) -Constants_D3*y[15];
    }

    inline data_type calc_Pitch_Dem_unsat(data_type& SBE_err, state_type& y) const
    {
        return Constants_P4*(SBE_err) + Constants_I4*y[16] + Constants_D4*Constants_N4*(SBE_err) -Constants_D4*y[17];
    }

    inline data_type calc_Ail_Dem_unsat(
        state_type& y, 
        data_type& BankAngle,
        const DerivedPlantData_Adv& derivedData
        ) const
        {
        data_type temp0 = sat(Constants_KRoll*(BankAngle-180/M_PI*y[6]),Constants_srate,-Constants_srate);
        return pid_Ail_Dem.Calc_PID_unsat(derivedData.scaler,temp0,y[9],y[13]);

    }

    inline data_type calc_Elev_Dem_unsat(
        state_type& y, 
        data_type& Pitch_Dem,
        const DerivedPlantData_Adv& derivedData
        ) const
        {
        data_type y7_rad_sat_80 = sat(y[6],80,-80);

        data_type Elev_Input= sat(Constants_K5*(Pitch_Dem-180/M_PI*y[7]),Constants_S5UP,Constants_S5DN)+Constants_C5*fabs(Constants_g*tan(y7_rad_sat_80)*sin(y7_rad_sat_80)/derivedData.Va)*cos(y[7])*180/M_PI;

        return pid_Elev_Dem.Calc_PID_unsat(derivedData.scaler,Elev_Input,y[10],y[18]);


    }

    inline data_type calc_Rudder_dem(
        state_type& y,
        data_type& Ail_Dem,
        const DerivedPlantData_Adv& derivedData
        ) const
        {
        data_type z6_Temp= Constants_g*Constants_Ctr_Yaw_YAW2SRV_RLL/std::max(derivedData.Va,(data_type)Constants_APlane_airspeed_min);

        return sat(-(y[20]-(y[11]-z6_Temp*tan(y[6])*cos(y[6])-y[19]))*Constants_K9*derivedData.scalerSq*180/M_PI+Constants_K10*Ail_Dem,45,-45);
    }



    virtual void DoNextStep(
        const data_type& chi_ci,
        const data_type& VTi,
        const data_type& HTi
    ) 
    {
        CalculateSignals(chi_ci,VTi,HTi);
        //std::cout<<"chi_c:"<<chi_c<<",HT:"<<HT<<",VT:"<<VT<<",phase_unwrap_chi_c.offset:"<<phase_unwrap_chi_c.offset<<",phase_unwrap_chi_c.previous_value:"<<phase_unwrap_chi_c.previous_value<<std::endl;
        BareDrone::CalculateAccelerations();
        BareDrone::integrate(*this);        
    } 

    inline void CalculateSignals
    (
        const data_type& chi_ci,
        const data_type& VTi,
        const data_type& HTi
    )
    {
        chi_c = chi_ci;
        VT = VTi;
        HT = HTi;
        data_type VTSq = sq(VT);
        chi_c = chi_ci+360;
        //std::cout<<"chi_ci:"<<chi_ci<<std::endl;
        //unwrapping chi_c
        chi_c = phase_unwrap_chi_c.unwrap(chi_c);

        //generating derived variables from  states
        DerivedPlantData_Adv derivedData(BareDrone::y,BareDrone::wind_vector);

        //unwrapping chi
        data_type chi = phase_unwrap_chi.unwrap(derivedData.chi*180/M_PI);

        //calculating bank angle a.k.a roll reference
        data_type error = chi_c - chi;
        data_type temp = Constants_P1*error+Constants_D1*Constants_N*error;
        data_type BankAngle = sat( temp - Constants_D1*BareDrone::y[12],45,-45);

        //Calculating Aileron Demand
        windup_with_Ail_Dem.unsat_signal = calc_Ail_Dem_unsat(BareDrone::y,BankAngle,derivedData);
               //data_type ail_dem = sat(windup_with_Ail_Dem.unsat_signal,45,-45);

        //Calculate STE err
        data_type STE_err = calc_STE_err(VTSq,HT,derivedData.VaSq,derivedData.h);
         
        //Calculate SBE err
        data_type SBE_err = calc_SBE_err(VTSq,HT,derivedData.VaSq,derivedData.h);
         
        //Calculating Throttle Demand
        windup_with_Throttle_Dem.unsat_signal = calc_Throttle_Dem_unsat(STE_err,BareDrone::y);
        //data_type throttle_dem = sat(windup_with_Throttle_Dem.unsat_signal,100,0);


        //Calculating Pitch Demand
        windup_with_Pitch_Dem.unsat_signal = calc_Pitch_Dem_unsat(SBE_err,BareDrone::y);
        data_type pitch_dem = sat(windup_with_Pitch_Dem.unsat_signal,30,-30);
         
        
        //Calculating Elev Demand
        windup_with_Elev_Dem.unsat_signal = calc_Elev_Dem_unsat(BareDrone::y,pitch_dem,derivedData);
        
        //data_type elev_dem = -sat(windup_with_Elev_Dem.unsat_signal,45,-45);

        // // Calculating Rudder Demand
        // data_type rudder_dem = calc_Rudder_dem(BareDrone::y,ail_dem,derivedData);
        

        data_type throttle = sat(BareDrone::y[21],100,0);
        data_type delta_ail = sat(BareDrone::y[22],30,-30);
        data_type delta_elev = sat(BareDrone::y[23],15,-15);
        data_type delta_rudd = sat(BareDrone::y[24],25,-25);
        // Calculate Force and Moments
        DroneWithCA::calculateFandM(derivedData,throttle,delta_ail,delta_elev,delta_rudd,BareDrone::Fxyz,BareDrone::Mxyz);
        Ab_y = Get_Ab_y (BareDrone::Fxyz[1]);

    }

    inline data_type Get_Ab_y(const data_type Fy){
        return BareDrone::y[5]*BareDrone::y[9]-BareDrone::y[3]*BareDrone::y[11]+Fy/BareDrone::Mass;
    }


    class DerivedPlantData_Adv: public DerivedPlantData<data_type> {

        
        public:
        typedef DerivedPlantData<data_type> parent;
        data_type scaler;
        data_type scalerSq;
        data_type h;
        data_type VaSq;
        DerivedPlantData_Adv(){std::cout<<"cocalled"<<std::endl;}
        ~DerivedPlantData_Adv(){}

        DerivedPlantData_Adv(state_type &y, const data_type (&wind_vector)[3]):DerivedPlantData<data_type>(y,wind_vector)
        {
            scaler = sat(Constants_Scailing_Speed/parent::Va,1.67,0.6);
            scalerSq = sq(scaler);
            h = -y[2];  
            VaSq = sq(parent::Va);  
        }

        private:
        explicit DerivedPlantData_Adv(const DerivedPlantData_Adv& other)
        {
            std::cout<<"called"<<std::endl;
        }

            //copy assignment constructor
        DerivedPlantData_Adv& operator=(const DerivedPlantData_Adv& other) 
        {
            std::cout<<"called"<<std::endl;
            return *this;
        }





    }; 
};




#endif