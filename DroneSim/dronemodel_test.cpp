#include <iostream>
#include <cstdlib>
#include <iomanip>
#include <cmath>
#include <ctime>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <dronemodel.hpp>
#include <phase_unwrapper.hpp>
#include <phase_wrapper.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include "DroneSim/derivedplantdata.hpp"
#include <boost/algorithm/string.hpp>

//OutputSinks
//#define _file_output
#define _stream_output



using namespace std;
typedef double data_type;
typedef Eigen::Matrix<data_type, 3, 1>  vector_type;
typedef Eigen::Matrix<data_type, 3, 3> matrix_type;



//declaring Helper functions
void print(data_type* y,int n);
void print(data_type t, data_type* y,int n);
void wrapToPi(data_type *p, int n);
bool checkPassCondition(data_type V1, data_type V2);
bool checkPassConditionPrecise(data_type V1, data_type V2);
//type defining state type for odeint use 

typedef std::vector< data_type > state_type;

int main()
{
  using namespace boost::numeric::odeint;

  typedef Drone<data_type,false,false> droneType;

  //declaring plant parameters
    vector_type Fxyz;
    vector_type Mxyz;
    data_type Mass =0;
    matrix_type I;
    data_type Ts = 0.0025;

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
    Euler_init<<2,3,1;
    
  //pqr initial
  //pqr = [1;1;2];
    vector_type pqr_init;
    pqr_init<<1,1,2;



  //Initializing Plant Parameters
  Mass =1.0;

      //Fxyz = [5;10;20];

  Fxyz<<5,10,20;
  //Fxyz=Fxyz*0;
  //Fxyz(2)=0;
    //Mxyz= [1;2;3];  
  Mxyz<<1,2,3;
  //Mxyz=Mxyz*0;
  I<<0.020000000000000,0,-0.01,
  0,0.026000000000000,0,
  -0.01,0,0.053000000000000;
  //data_type& Ts,data_type& Mass, matrix_type& I,vector_type& Xe_init, vector_type& Vb_init, vector_type& Euler_init, vector_type& Euler_dot_init
  droneType drone(Ts,Mass,Xe_init,Vb_init,Euler_init,pqr_init);


  clock_t begin = clock();
  //integrating 
  //integrate_const(make_dense_output<dopri_stepper_type>( abserr, relerr ),get_y_dot, y, 0.0, 15.0, dt , streaming_observer(std::cout,outputFile));
  for(data_type t = Ts; t <=15; t+=Ts)
  {
    drone.DoNextStep({Fxyz(0),Fxyz(1),Fxyz(2)},{Mxyz(0),Mxyz(1),Mxyz(2)});
    //std::cout<<drone.y.at(0)<<std::endl;
  }
  std::cout<<drone.y.at(0)<<std::endl;
  std::cout<<drone.y.at(1)<<std::endl;
  std::cout<<drone.y.at(2)<<std::endl;
  //recording the integration stop time
  clock_t end = clock();
  data_type elapsed_secs = data_type(end - begin) / CLOCKS_PER_SEC;

  //writing time taken for integration
  cout<<"time(s):"<<elapsed_secs<<endl;


  Drone<data_type,false,true> drone2(Ts,Mass,Xe_init,Vb_init,Euler_init,pqr_init);
  //drone2.DoNextStep(Fxyz,Mxyz);

  
//*****************************Creating data for plotting with control allocator***********************************

	std::ifstream file;
  std::ofstream ofile;
  //file.open("/home/geo/Dropbox/Thesis/C++/TAERInput.Csv");
  file.open("TAERInput.Csv");
  ofile.open("TAERCplusplus_output.Csv");
	std::string line = "";
	// Iterate through each line and split the content using delimeter
	begin = clock();
  int count =0;
  while (std::getline(file, line))
	{
      ofile<<drone2.t<<",";
      for(size_t i = 0; i < 12; i++)
      {
        ofile<<drone2.y[i]<<((i==11)?"\n":",");
      }
    count++;
		std::vector<std::string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(","));
    /*std::cout<<"time:"<<vec[0]<<std::endl;
    std::cout<<"time:"<<vec[1]<<std::endl;
    std::cout<<"time:"<<vec[2]<<std::endl;
    std::cout<<"time:"<<vec[3]<<std::endl;
    std::cout<<"time:"<<vec[4]<<std::endl;*/
     drone2.wind_vector[0]= atof(vec[6].c_str());
     drone2.wind_vector[1]= atof(vec[7].c_str());
     drone2.wind_vector[2]= atof(vec[8].c_str());
     //std::cout<<drone2.wind_vector[0]<<","<<drone2.wind_vector[1]<<","<<drone2.wind_vector[2]<<std::endl;
		 drone2.DoNextStep(
      atof(vec[1].c_str()),
      atof(vec[2].c_str()),
      atof(vec[3].c_str()),
      atof(vec[4].c_str())
      );

      //std::cout<<drone2.y.at(0)<<std::endl;
      /*std::cout<<drone2.y.at(1)<<std::endl;
      std::cout<<drone2.y.at(2)<<std::endl;
      std::cout<<drone2.y.at(3)<<std::endl;
      std::cout<<drone2.y.at(4)<<std::endl;
      std::cout<<drone2.y.at(5)<<std::endl;
      std::cout<<"Euler"<<drone2.y.at(6)<<std::endl;
      std::cout<<drone2.y.at(7)<<std::endl;
      std::cout<<drone2.y.at(8)<<std::endl;
      std::cout<<"Euler_dot"<<drone2.y.at(9)<<std::endl;
      std::cout<<drone2.y.at(10)<<std::endl;
      std::cout<<drone2.y.at(11)<<std::endl; */
      wrapToPi(&(drone2.y[6]),3);
      //print(&(drone2.y[6]),3);
      //std::cout<<"time:"<<drone2.t<<std::endl; 


      
	}
	// Close the File
	file.close();
  ofile.close();
  end = clock();
  elapsed_secs = data_type(end - begin) / CLOCKS_PER_SEC;
  cout<<"Count of data points:"<<count<<std::endl;
  cout<<"time with control allocator(s):"<<elapsed_secs<<endl;
//*****************************Testing dy function***********************************
  std::cout<<"Testing dy function"<<std::endl;

 file.open("Dy_Test_Data.Csv");
 line = "";
	// Iterate through each line and split the content using delimeter
  count =0;
  begin = clock();
	while (std::getline(file, line))
	{
    count++;
		std::vector<std::string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(","));
    state_type y = state_type(12);
    state_type dy = state_type(12);
    for(size_t i = 0; i < 12; i++)
    {
      y[i] = atof(vec[i].c_str());
    }
    

    data_type F[3] = {atof(vec[12].c_str()),atof(vec[13].c_str()), atof(vec[14].c_str())};
    data_type Mo[3] = {atof(vec[15].c_str()),atof(vec[16].c_str()), atof(vec[17].c_str())};

    drone.get_y_dot(y,dy,0,DerivedPlantData<data_type>(y,{0,0,0}),F,Mo);
    bool failed = false;


    for(size_t i = 0; i < 12; i++)
    {
      //std::cout<<dy[i]-atof(vec[18+i].c_str())<<((i==11)?"\n":",");
      if(abs(dy[i]-atof(vec[18+i].c_str()))>0.0000000001)
      {
        std::cout<<dy[i]-atof(vec[18+i].c_str())<<std::endl;
        std::cout<<atof(vec[18+i].c_str())<<std::endl;
        std::cout<<std::endl<<"****dyFailed****"<<std::endl;
        std::cout<<std::endl<<"state:"<<i<<std::endl;
        failed = true;
        break;
      }
    }
    if(failed==true) {break;}

    //break;
	}

  end = clock();
  file.close();


  elapsed_secs = data_type(end - begin) / CLOCKS_PER_SEC;
  cout<<"Count of data points:"<<count<<std::endl;
  cout<<"dy time(s):"<<elapsed_secs<<endl;

//*****************************Testing derived Data function***********************************
  std::cout<<"Testing derived Data function"<<std::endl;

 file.open("DerivedData_Test_Data.Csv");
 line = "";
 begin = clock();
 count = 0;
  // Iterate through each line and split the content using delimeter
	while (std::getline(file, line))
	{
    count++;
		std::vector<std::string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(","));
    state_type y = state_type(12);
    for(size_t i = 0; i < 12; i++)
    {
      y[i] = atof(vec[i].c_str());
    }
    
    data_type windV[3] = {atof(vec[12].c_str()),atof(vec[13].c_str()),atof(vec[14].c_str())};
    //droneType::DerivedPlantData dData(y,windV);
    DerivedPlantData<data_type> dData2(y,windV);
    bool failed = false;

    failed =checkPassCondition(dData2.Va,atof(vec[15].c_str()));
    if(failed==true) {cout<<"Va Failed"<<endl;break;}
    failed =checkPassCondition(dData2.alpha,atof(vec[16].c_str()));
    if(failed==true) {cout<<"alpha Failed"<<endl;break;}
    failed =checkPassCondition(dData2.beta,atof(vec[17].c_str()));
    if(failed==true) {cout<<"beta Failed"<<endl;break;}
    failed =checkPassCondition(dData2.q_bar,atof(vec[18].c_str()));
    if(failed==true) {cout<<"q bar Failed"<<endl;break;}
    failed =checkPassCondition(dData2.chi,atof(vec[19].c_str()));
    if(failed==true) {cout<<"chi Failed"<<endl;break;}

    //break;
	}

  end = clock();
  file.close();


  elapsed_secs = data_type(end - begin) / CLOCKS_PER_SEC;
  cout<<"Count of data points:"<<count<<std::endl;
  cout<<"derived data time(s):"<<elapsed_secs<<endl;


//*****************************Testing Drone with low level Controller Basics***********************************
  std::cout<<"Testing Drone with low level Controller Basics"<<std::endl;
  Drone<data_type,true,true> drone3(Ts,Mass,Xe_init,Vb_init,Euler_init,pqr_init);

//*****************************Testing Phase Unwrapper***********************************
  std::cout<<"Testing Phase Unwrapper"<<std::endl;
  phase_unwrapper<float,true> phase_unwrap_test;

  checkPassCondition(phase_unwrap_test.unwrap(115),115);
  checkPassCondition(phase_unwrap_test.unwrap(175),175);
  
  checkPassCondition(phase_unwrap_test.unwrap_temp(-180),180);
  checkPassCondition(phase_unwrap_test.unwrap(175 ),175);
  checkPassCondition(phase_unwrap_test.unwrap(-175),185);
  checkPassCondition(phase_unwrap_test.unwrap_temp(-172.5),187.5);
  checkPassCondition(phase_unwrap_test.unwrap(-170),190);
  checkPassCondition(phase_unwrap_test.unwrap(0   ),360);
  checkPassCondition(phase_unwrap_test.unwrap(15  ),375);
  checkPassCondition(phase_unwrap_test.unwrap(175 ),535);
  checkPassCondition(phase_unwrap_test.unwrap(-175),545);
  checkPassCondition(phase_unwrap_test.unwrap(0   ),720);
  checkPassCondition(phase_unwrap_test.unwrap(-175),545);
  checkPassCondition(phase_unwrap_test.unwrap(175 ),535);
  checkPassCondition(phase_unwrap_test.unwrap(0   ),360);
  checkPassCondition(phase_unwrap_test.unwrap(-15 ),345);
  checkPassCondition(phase_unwrap_test.unwrap(-175),185);
  checkPassCondition(phase_unwrap_test.unwrap(175 ),175);
  checkPassCondition(phase_unwrap_test.unwrap(0   ),0);
  checkPassCondition(phase_unwrap_test.unwrap(-175   ),-175);
  checkPassCondition(phase_unwrap_test.unwrap_temp(175   ),-185);
  checkPassCondition(phase_unwrap_test.unwrap(-175   ),-175);
  checkPassCondition(phase_unwrap_test.unwrap(175   ),-185);
  checkPassCondition(phase_unwrap_test.unwrap(0   ),-360);
  checkPassCondition(phase_unwrap_test.unwrap(-180   ),-540);
  checkPassCondition(phase_unwrap_test.unwrap(175   ),-545);
  checkPassCondition(phase_unwrap_test.unwrap(0   ),-720);
  checkPassCondition(phase_unwrap_test.unwrap(-175   ),-895);

//*****************************Testing dy function in drone (full)***********************************
  std::cout<<"Testing dy function in drone (full)"<<std::endl;
  file.open("Dy_lo_Test_Data.Csv");
 line = "";
 begin = clock();
 count = 0;
	// Iterate through each line and split the content using delimeter
	while (std::getline(file, line))
	{
    count++;
    if(count!=7655)
      continue;
		std::vector<std::string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(","));
    for(size_t i = 0; i < 13; i++)
    {
      drone3.y[12+i] = atof(vec[i].c_str());

    }
    for(size_t i = 0; i < 12; i++)
    {
      drone3.y[i] = atof(vec[13+i].c_str());
    }

    for(size_t i = 0; i < 25; i++)
    {
    //std::cout<<drone3.y[i]<<std::endl;
    }
    
    


    state_type yp(25);
    
    std::cout.precision(20);
    data_type chi_c =  atof(vec[25].c_str());
    data_type VT = atof(vec[26].c_str());
    data_type HT = atof(vec[27].c_str());
    data_type windV[3] = {atof(vec[28].c_str()),atof(vec[29].c_str()),atof(vec[30].c_str())};
    
    Drone<data_type,true,true>::DerivedPlantData_Adv derivedData(drone3.y,windV);

        data_type throttle = sat(drone3.y[21],100,0);
        data_type delta_ail = sat(drone3.y[22],30,-30);
        data_type delta_elev = sat(drone3.y[23],15,-15);
        data_type delta_rudd = sat(drone3.y[24],25,-25);
        // Calculate Force and Moments
        drone3.calculateFandM(derivedData,throttle,delta_ail,delta_elev,delta_rudd,drone3.Fxyz,drone3.Mxyz);
        drone3.Ab_y = drone3.Get_Ab_y (drone3.Fxyz[1]);
        
    //std::cout<<windV[0]<<","<<windV[1]<<","<<windV[2]<<std::endl;
    drone3.get_y_dot(drone3.y,yp,0,derivedData,chi_c,VT,HT);

    for(size_t i = 0; i < 13; i++)
    {
      bool result = checkPassConditionPrecise(atof(vec[31+i].c_str()),yp[12+i]);
      if(result)
      {
        std::cout<<"****Failed@LoCo"<<i<<std::endl;
        std::cout<<"count:"<<count<<std::endl;
        std::cout<<"line:"<<line<<std::endl;
      }
      
    }
    for(size_t i = 0; i < 12; i++)
    {
      bool result = checkPassConditionPrecise(atof(vec[44+i].c_str()),yp[i]);
      if(result)
      {
        std::cout<<"Failed@Drone"<<i<<std::endl;
      }
    }

    //     for(size_t i = 0; i < 13; i++)
    // {
    //   std::cout<<"yp["<<1+i<<"]="<<yp[12+i]<<std::endl;
    // }

    // for(size_t i = 0; i < 12; i++)
    // {
    //   std::cout<<"yp["<<1+13+i<<"]="<<yp[i]<<std::endl;
    // }

    //break;

	}
  cout<<"Count of data points:"<<count<<std::endl;
  end = clock();
  file.close();
  
 /* //*****************************Testing wrap*********************************** */
  std::cout<<"Testing wrap "<<std::endl;
  phase_wrap<double> wrapPosNegPi(-M_PI,M_PI);
   std::cout<<"wraptest1.5PI:"<<wrapPosNegPi.wrap(1.5*M_PI)<<std::endl;
   std::cout<<"wraptest-1.5PI:"<<wrapPosNegPi.wrap(-1.5*M_PI)<<std::endl;
   std::cout<<"wraptest2.5PI:"<<wrapPosNegPi.wrap(2.5*M_PI)<<std::endl;
   std::cout<<"wraptest-2.5PI:"<<wrapPosNegPi.wrap(-2.5*M_PI)<<std::endl;
   std::cout<<"wraptest3.5PI:"<<wrapPosNegPi.wrap(3.5*M_PI)<<std::endl;
   std::cout<<"wraptest-1.5PI:"<<wrapPosNegPi.wrap(-3.5*M_PI)<<std::endl;
   std::cout<<"wraptest0PI:"<<wrapPosNegPi.wrap(0)<<std::endl;
   std::cout<<"wraptest-PI:"<<wrapPosNegPi.wrap(-M_PI)<<std::endl;
   std::cout<<"wraptestPI:"<<wrapPosNegPi.wrap(M_PI)<<std::endl;

 /* //*****************************Testing full drone *********************************** */
  std::cout<<"Testing full drone "<<std::endl;
  file.open("chi_c_VT_HT_Data.Csv");
  ofile.open("chi_c_VT_HT_output.Csv");
    vector_type Euler_init2;
    Euler_init2<<0.1,0.1,1;

    Drone<data_type,true,true> drone4(Ts,Mass,Xe_init,Vb_init,Euler_init2,pqr_init);


  line = "";
  begin = clock();
  count = 0;
	  // Iterate through each line and split the content using delimeter
	while (std::getline(file, line))
	{
      ofile<<drone4.t<<",";
      for(size_t i = 0; i < 13; i++)
      {
        ofile<<drone4.y[12+i]<<",";
      }
      for(size_t i = 0; i < 12; i++)
      {
        ofile<<drone4.y[i]<<((i==11)?"\n":",");
      }
    
    count++;
		std::vector<std::string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(","));
    drone4.wind_vector[0]= atof(vec[5].c_str());
    drone4.wind_vector[1]= atof(vec[6].c_str());
    drone4.wind_vector[2]= atof(vec[7].c_str());
    drone4.DoNextStep(atof(vec[1].c_str()),atof(vec[2].c_str()),atof(vec[3].c_str()));

    wrapPosNegPi.wrapVector3(&(drone4.y[6]));


    //break;
	}

  end = clock();
    elapsed_secs = data_type(end - begin) / CLOCKS_PER_SEC;
  cout<<"Count of data points:"<<count<<std::endl;
  cout<<"drone full testing time(s):"<<elapsed_secs<<endl;

  file.close();
  ofile.close();
 /* //*****************************Testing full drone 2 *********************************** */
  std::cout<<"Testing full drone 2 "<<std::endl;

  ofile.open("chi_c_VT_HT_output_2.Csv");


    Drone<data_type,true,true> drone5(Ts,Mass,Xe_init,Vb_init,Euler_init2,pqr_init);

  data_type chi_c = M_PI/2;
  data_type HT = 50;
  data_type VT = 15;

  line = "";
  begin = clock();
  count = 0;
	  // Iterate through each line and split the content using delimeter
	while (drone5.t<=15)
	{
      ofile<<drone5.t<<",";
      for(size_t i = 0; i < 13; i++)
      {
        ofile<<drone5.y[12+i]<<",";
      }
      for(size_t i = 0; i < 12; i++)
      {
        ofile<<drone5.y[i]<<((i==11)?"\n":",");
      }
    
    count++;

    //drone5.wind_vector[0]= 0;
    //drone5.wind_vector[1]= 0;
    //drone5.wind_vector[2]= 0;
    //std::cout<<drone5.y[0]<<","<<drone5.y[1]<<std::endl;
    drone5.DoNextStep(chi_c*180/M_PI,VT,HT);

    wrapPosNegPi.wrapVector3(&(drone5.y[6]));


    //break;
	}

  end = clock();
    elapsed_secs = data_type(end - begin) / CLOCKS_PER_SEC;
  cout<<"Count of data points:"<<count<<std::endl;
  cout<<"drone full testing time(s):"<<elapsed_secs<<endl;

  ofile.close();



return 0;
//*****************************Testing dy function for a special case ***********************************
  std::cout<<"Testing dy function for a special case"<<std::endl;

  begin = clock();
  count = 0;
  Drone<data_type,true,true> drone6(Ts,Mass,Xe_init,Vb_init,Euler_init,pqr_init);
  drone6.y[12]=-11865.8418875993;
  drone6.y[13]=1.71530192937419;
  drone6.y[14]=58.1576016311745;
  drone6.y[15]=-12.3131152893997;
  drone6.y[16]=-271.274611753465;
  drone6.y[17]=-3835.39407889643;
  drone6.y[18]=-9.91293999214427;
  drone6.y[19]=0.721000185994219;
  drone6.y[20]=0.0559791623481106;
  drone6.y[21]=3.07254678310586E-025;
  drone6.y[22]=2.81661936959798;
  drone6.y[23]=44.9999999999992;
  drone6.y[24]=44.9999729286522;
  drone6.y[0]=106.014651315678;
  drone6.y[1]=452.461914968843;
  drone6.y[2]=-54.1866385969535;
  drone6.y[3]=11.1949059819299;
  drone6.y[4]=2.0739126175533;
  drone6.y[5]=-1.51503775588149;
  drone6.y[6]=-0.809595290897756;
  drone6.y[7]=2.84727885237375;
  drone6.y[8]=-2.88897635934125;
  drone6.y[9]=0.26166364856416;
  drone6.y[10]=-0.135063915184218;
  drone6.y[11]=0.486512594243228;
  drone6.phase_unwrap_chi.previous_value = 12.868367661522030;
  drone6.phase_unwrap_chi.offset = 3.599999999999989e+02;
  drone6.phase_unwrap_chi.unwrap(12.868367661522030);
  data_type chi_ci = -81.7710882414561;
  data_type VTi = 14.0762641175854;
  data_type HTi = 49.0762641176;
  state_type yp(25);
  drone6.CalculateSignals(chi_ci,VTi,HTi);
  drone6.get_y_dot(drone6.y,yp,0,Drone<data_type,true,true>::DerivedPlantData_Adv(drone6.y,drone6.wind_vector),drone6.chi_c,drone6.VT,drone6.HT);
    cout<<"Count of data points:"<<count<<std::endl;
    end = clock();
    
  
  
    return 0;
   }
  
  
  //
  
  //Helper Functions Imlementation
  //Function to wrap angles
  void wrapToPi(data_type *p, int n)
  {
  
  for(int i =0;i<n;i++)
  {
    while(p[i]>M_PI)
    {
      p[i] = p[i] - 2*M_PI;
    }
    while(p[i]<-M_PI)
    {
      p[i] = p[i] + 2*M_PI;
    }
  }
  
}
//Function to print a given array
void print(data_type* y,int n)
{
  using namespace std;

  for(int i=0;i<n;i++)
  {
    cout<<y[i]<<" ";
  }
  cout<<endl;
}
//Function to print a given array with time
void print(data_type t, data_type* y,int n)
{
  using namespace std;
  cout<<t<<" ";
  print(y,n);
}

bool checkPassCondition(data_type V1, data_type V2)
{
  if(abs(V1-V2)>0.000001)
  {
    std::cout<<"V1:"<<V1<<std::endl;
    std::cout<<"V2:"<<V2<<std::endl;
    std::cout<<"****Failed****"<<std::endl;
    return true;
  }
  return false;
}

bool checkPassConditionPrecise(data_type V1, data_type V2)
{
  if(abs(V1-V2)/abs(V1)*100>0.00000001)
  {
    std::cout<<"****Failed****"<<std::endl;
    std::cout<<V1<<std::endl;
    std::cout<<V2<<std::endl;
    std::cout<<abs(V1-V2)/abs(V1)*100<<std::endl;
    return true;
  }
  return false;
}
