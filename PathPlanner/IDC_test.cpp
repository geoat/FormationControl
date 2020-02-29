#include "IDC.hpp"

    typedef double data_type;
    typedef Eigen::Matrix<data_type, 3, 1>  vector_type;
    typedef Eigen::Matrix<data_type, 3, 3> matrix_type;
    typedef std::vector<data_type> state_type;


int main()
{
    data_type Ts = 0.0025;

     data_type Mass =0;
    matrix_type I;
    //data_type Ts = 0.0025;

  //setting intial conditions for states
  //xyz initial
  //xyz = [0,500,-50]';

    vector_type Xe_init;
    Xe_init<<0,500,-50;

  //uvw initial
  //uvw = [15,0,0]';
    vector_type Vb_init;
    Vb_init<<15,0,0;
  //euler initial
  //euler = [2;3;1];
    vector_type Euler_init;
    Euler_init<<0.1,0.1,1;
    
  //pqr initial
  //pqr = [1;1;2];
    vector_type pqr_init;
    pqr_init<<1,1,2;



  //Initializing Plant Parameters
    Mass =1.0;

    Drone<data_type,false,false> drone(Ts,Mass,Xe_init,Vb_init,Euler_init,pqr_init);


    IDC<double> controller(drone);
    DerivedPlantData<data_type> dData(drone.y,{0,0,0});
    state_type trajectory(12);
    DerivedPlantData<data_type> trajectoryData(trajectory,{0,0,0});
    state_type acceleration(3);
    state_type wbdot(3);
    controller.CalculateFandM(drone,dData,trajectory,trajectoryData,acceleration,wbdot);
}