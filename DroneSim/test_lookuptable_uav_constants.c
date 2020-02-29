#include "uav_constants.hpp"
#include <iostream>
#include <vector>
#include "lookuptable.hpp"




int main()
{
    std::cout<<"************Test1*************"<<std::endl;
    //Test with junk arrays seperate kf and interpolation test for Elements_Array
    Elements_Array<float> object = {-1,3,5,7,8};

    for(float i = -2; i < 15; i+=0.5)
    {
        
            k_f_pair<float> pair = object.get_k_f(i);
            std::cout<<"value:"<<i<<";k:"<<pair.k<<";f:"<<pair.f<<std::endl;
            std::cout<<"interpolate:"<<object.interpolate(pair)<<std::endl;        
    }

    std::cout<<"************Test2*************"<<std::endl;
    //Test for OneDLookUpTable with multiple outputs
    Elements_Array<float> object2 = {-1,3,5,7,9};
    for(float i = -2; i < 15; i+=0.5)
    {
            OneDLookUpTable<float,float,2> test(object,object,object2);
            float result[2];
            std::cout<<"Input Value :"<<i<<std::endl;
            test.interpolate(result,i);
            for(int j = 0; j < 2; j++)
            {
                std::cout<<"Output Value "<<j<<":"<<result[j]<<std::endl;
            }
            
    }

    std::cout<<"************Test3*************"<<std::endl;
    //Test for OneDLookUpTable with aerodynamic coefficients - Compare With Matlab
    auto alpha_array = Elements_Array<float>(Constants_Aero_alpha_vector);
    OneDLookUpTable<float,float,9> alpha_lookup(alpha_array,Elements_Array<float>(Constants_Aero_CD_Basic),Elements_Array<float>(Constants_Aero_Cy_RollRate),Elements_Array<float>(Constants_Aero_CL_Basic),Elements_Array<float>(Constants_Aero_Cl_Beta),Elements_Array<float>(Constants_Aero_Cl_RollRate),Elements_Array<float>(Constants_Aero_Cl_YawRate),Elements_Array<float>(Constants_Aero_Cm_Basic),Elements_Array<float>(Constants_Aero_Cn_RollRate),Elements_Array<float>(Constants_Aero_Cn_YawRate));
    float alpharesult[9];
    float input = 4.39;
    alpha_lookup.interpolate(alpharesult,input);
    std::cout<<"Input Value :"<<input<<std::endl;
    for(int i = 0; i < 9; i++)
    {
        std::cout<<"Output Value "<<i<<":"<<alpharesult[i]<<std::endl;
    }
    
    // 
    auto a = TwoDOutputValues<float>(Constants_Aero_Cn_ail);
    auto b(a);
    TwoDOutputValues<float> c;
    c = b;
    //float array[][1] = Constants_Aero_Cn_YawRate;
    //std::cout<<"program end:"<<std::endl;

    std::cout<<"************Test4*************"<<std::endl;
    //test for testing TwoDOutputValues
    auto TwoDOtest2 = TwoDOutputValues<float>({{1,2},{4,3}});
    std::cout<<TwoDOtest2.interpolate(k_f_pair<float>(0,0.75),k_f_pair<float>(0,0.25))<<std::endl;
    std::cout<<TwoDOtest2.interpolate(k_f_pair<float>(0,0.25),k_f_pair<float>(0,0.75))<<std::endl;
    std::cout<<TwoDOtest2.interpolate(k_f_pair<float>(0,0),k_f_pair<float>(0,0))<<std::endl;
    std::cout<<TwoDOtest2.interpolate(k_f_pair<float>(0,1),k_f_pair<float>(0,0))<<std::endl;
    std::cout<<TwoDOtest2.interpolate(k_f_pair<float>(0,0),k_f_pair<float>(0,1))<<std::endl;
    std::cout<<TwoDOtest2.interpolate(k_f_pair<float>(0,1),k_f_pair<float>(0,1))<<std::endl;
    std::cout<<TwoDOtest2.interpolate(k_f_pair<float>(0,1.5),k_f_pair<float>(0,1))<<std::endl;
    std::cout<<TwoDOtest2.interpolate(k_f_pair<float>(0,1.5),k_f_pair<float>(0,2.5))<<std::endl;

    auto TwoDOtest22 = TwoDOutputValues<float>({{1,13,3,8},{9,14,16,12},{5,6,7,4},{2,10,15,11}});
    std::cout<<TwoDOtest22.interpolate(k_f_pair<float>(1,0.5),k_f_pair<float>(1,0.5))<<std::endl;
    std::cout<<TwoDOtest22.interpolate(k_f_pair<float>(2,0.5),k_f_pair<float>(2,0.5))<<std::endl;
    std::cout<<TwoDOtest22.interpolate(k_f_pair<float>(2,1.5),k_f_pair<float>(2,0.5))<<std::endl;
    std::cout<<TwoDOtest22.interpolate(k_f_pair<float>(0,1.5),k_f_pair<float>(0,0.5))<<std::endl;

    std::cout<<"************Test5*************"<<std::endl;
    //test for TwoDLookUpTable
    Elements_Array<float> TwoDX = {-1,3,5,7};
    Elements_Array<float> TwoDY = {1,2,3,10};

    auto TwoDLookUpTableTest = TwoDLookUpTable<float,float>(TwoDX,TwoDY,TwoDOtest22);
    float xvalue = 4;
    float yvalue = 1;
    std::cout<<xvalue<<","<<yvalue<<":"<<TwoDLookUpTableTest.interpolate(xvalue,yvalue)<<std::endl;
    xvalue = 0;
    yvalue = 0;    
    std::cout<<xvalue<<","<<yvalue<<":"<<TwoDLookUpTableTest.interpolate(xvalue,yvalue)<<std::endl;
    xvalue = 9;
    yvalue = 2.5;    
    std::cout<<xvalue<<","<<yvalue<<":"<<TwoDLookUpTableTest.interpolate(xvalue,yvalue)<<std::endl;

    std::cout<<"************Test6*************"<<std::endl;
    //test for TwoDLookUpTable with aerocoefficeints - Compare with matlab computation

    auto TwoDLookUpTableTestAero = TwoDLookUpTable<float,float>(alpha_array,Elements_Array<float>(Constants_Aero_delta_ail_vector),TwoDOutputValues<float>(Constants_Aero_Aero_CD_elev));
    xvalue = 1.5;
    yvalue = 2;  
    std::cout<<xvalue<<","<<yvalue<<":"<<TwoDLookUpTableTestAero.interpolate(xvalue,yvalue)<<std::endl;

    xvalue = 10;
    yvalue = 15;  
    std::cout<<xvalue<<","<<yvalue<<":"<<TwoDLookUpTableTestAero.interpolate(xvalue,yvalue)<<std::endl;


    xvalue = -1;
    yvalue = 3.75;  
    std::cout<<xvalue<<","<<yvalue<<":"<<TwoDLookUpTableTestAero.interpolate(xvalue,yvalue)<<std::endl;

    xvalue = 50;
    yvalue = 50;  
    std::cout<<xvalue<<","<<yvalue<<":"<<TwoDLookUpTableTestAero.interpolate(xvalue,yvalue)<<std::endl;

    return 0;
}