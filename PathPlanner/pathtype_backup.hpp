#ifndef PATHTYPE_HPP
#define PATHTYPE_HPP

#include <vector>
#include <exception>
#include <memory>

#include "point.hpp"
#include "DroneSim/derivedplantdata.hpp"
#include "vg_estimator.hpp"
#include "pathcalculationdata.hpp"

template <typename data_type>
class Path
{
    
    public:
    virtual data_type GetChi_c(
        const DerivedPlantData<data_type>& derivedData, 
        const Point<data_type>& location,
        const data_type* Vb
        ) 
    {
        return 0;
    }
    virtual std::unique_ptr<Path<data_type>> GetMission(const Point<data_type>& location){}
    #ifdef PATHTYPE_HPP_TEST
    virtual void GetNextLocation(Point<data_type>& location)
    {
        std::cout<<"Base"<<std::endl;


    }
    #endif
};

template <typename data_type>
class Line: public Path<data_type>
{
    public:
    Point<data_type> path_origin;
    Point<data_type> slope;



    LineVgEstimator<data_type> lineVgEstimator;
    data_type Ts;
    Line()
    {

    }

    Line(
        const Point<data_type>& origin, 
        const Point<data_type>& slope
        ):path_origin(origin),slope(slope)
    {

    }
    template<typename T>
    Line(
        const Point<data_type>& origin, 
        const Point<data_type>& slope, 
        const T&  Ts
        ):path_origin(origin),slope(slope)
    {
        this->Ts = (data_type)Ts;
        this->lineVgEstimator.Ts = this->Ts;
    }
    ~Line(){/*std::cout<<"LDcalled"<<std::endl;*/}

    virtual std::unique_ptr<Path<data_type>>  GetMission(const Point<data_type>& location) override
    {
        return std::unique_ptr<Path<data_type>>(this);
    }

    virtual data_type GetChi_c(
        const DerivedPlantData<data_type>& derivedData, 
        const Point<data_type>& location,
        const data_type* Vb)
    {
        LineCalculationData<data_type> LineCalculationData(this,derivedData,location);   
        Vector3<data_type> Vb_vec(Vb);     
        data_type Vg_hat = this->lineVgEstimator.GetVgHat(&LineCalculationData,derivedData,location,Vb_vec);
        //std::cout<<Vg_hat<<","<<derivedData.Va<<",";
        //Vg_hat = derivedData.Va;


        		/*%% Parameters
		epsi = param(1);
		chi_inf = param(2);
		k = param(3);
		kk = param(4);
		alpha = param(5);

		%%
		term =(chi-chi_d)/epsi;
		if abs(term)<=1
			chi_tilde_norm=term;

		else
			chi_tilde_norm=sign(term);
		end

		term1 = chi;
		term2 = -1*chi_inf*2/pi/alpha*k*(sin(chi-chi_q))/(1+(k*(e))^2)*Vg;
		term3 = -kk/alpha*chi_tilde_norm;

		chi_c= term1 + term2 + term3;*/

        float term = (derivedData.chi - LineCalculationData.chi_d)/LineVFParameters<data_type>::line_epsi;
        float sat_term = term;
        if(fabs(term)>1)
        {
            sat_term = sign(term);
        }

        float term2 = -LineVFParameters<data_type>::chi_inf*TWO_OVER_M_PI/LineVFParameters<data_type>::est_alpha*LineVFParameters<data_type>::line_k*sin(derivedData.chi-LineCalculationData.chi_q)/(1+sq(LineVFParameters<data_type>::line_k*LineCalculationData.e_y))*Vg_hat;
        float term3 = -LineVFParameters<data_type>::line_kk*sat_term/LineVFParameters<data_type>::est_alpha;
        float chi_c =  derivedData.chi  + term2 + term3;

        return chi_c;
    }

    #ifdef PATHTYPE_HPP_TEST
    void GetNextLocation(Point<data_type>& location) override
    {
        //std::cout<<"Line"<<std::endl;
        location = location+slope*0.01;
    }
    #endif

};

template <typename data_type>
class Orbit: public Path<data_type>
{
    public:
    data_type Ts;
    Point<data_type> centre;
    data_type radius = 0;
    //direction
    data_type direction = -1;
    OrbitVgEstimator<data_type> orbitVgEstimator;
    Orbit(){}
    Orbit(
        const data_type& radius, 
        const Point<data_type>& centre
        ):radius(radius),centre(centre)
    {

    }

    template<typename T>
    Orbit(
        const data_type& radius, 
        const Point<data_type>& centre,
        const T&  Ts
        ):Orbit(radius,centre)
    {
        this->Ts = (data_type)Ts;
        this->orbitVgEstimator.Ts = this->Ts;
    }


    ~Orbit(){}
    virtual std::unique_ptr<Path<data_type>>  GetMission(const Point<data_type>& location) override
    {
        return std::unique_ptr<Path<data_type>>(this);
    }

    virtual data_type GetChi_c(
        const DerivedPlantData<data_type>& derivedData, 
        const Point<data_type>& location,
        const data_type* Vb
        )
    {

        OrbitCalculationData<data_type> OrbitCalculationData(this,derivedData,location);
        Vector3<data_type> Vb_vec(Vb);     
        data_type Vg_hat = this->orbitVgEstimator.GetVgHat(&OrbitCalculationData,derivedData,location,Vb_vec);
        //Vg_hat = derivedData.Va;
        //std::cout<<"Vg_hat:"<<Vg_hat<<",";
/*         %%
        k = param(1);
        kk = param(2);
        epsi = param(3);
        r = param(4);
        j = param(5);
        alpha = param(6); */

        // %%
        // chi_tilde = chi-chi_d;
        data_type chi_tilde = derivedData.chi-OrbitCalculationData.chi_d;

        // lt = chi_tilde/epsi;
        data_type lt = chi_tilde/OrbitVFParameters<data_type>::epsi;
        // if abs(lt) <=1
        //     term = lt;
        // else
        //     term = sign(lt);
        // end
        data_type term = lt;
        if (fabs(lt)>1)
            term = sign(lt);

        // d_tilde = d-r;
        data_type d_tilde = OrbitCalculationData.d-OrbitCalculationData.orbit->radius;

        // beta = k/(1+(k*d_tilde)^2);
        data_type beta = OrbitVFParameters<data_type>::k/(1+sq(OrbitVFParameters<data_type>::k*d_tilde));
        // err = (chi-gamma);
        data_type err = derivedData.chi - OrbitCalculationData.gamma;

        // term2 = Vg /(alpha*d)*sin(err) + j*beta/alpha*Vg*cos(err);
        data_type term2 = Vg_hat /(VFParameters<data_type>::est_alpha*OrbitCalculationData.d)*sin(err) + OrbitCalculationData.orbit->direction*beta/VFParameters<data_type>::est_alpha*Vg_hat*cos(err);
        // term3 = -kk/alpha*term;
        data_type term3 = -OrbitVFParameters<data_type>::kk/VFParameters<data_type>::est_alpha*term;
        // y = chi + term2 + term3;
        return derivedData.chi + term2 + term3;
    }

    #ifdef PATHTYPE_HPP_TEST
    void GetNextLocation(Point<data_type>& location) override
    {
        //std::cout<<"Orbit"<<std::endl;
        location.rotate(this->direction*0.00001,this->centre);
    }
    #endif


};

template <typename data_type>
class Way: public Path<data_type>
{
    public: 

    std::vector<Point<data_type>> way_points;

    Way()
    {
        way_points.reserve(100);

    }
    template<typename T>
    Way(const T& radius):Way()
    {
        this->radius = (data_type)radius;
   
        
    }


    template<typename T, typename ...Us>
    Way(const T& radius, Us ... points):Way(radius)
    {
        if(sizeof...(points)<3) throw "no enough waypoints";
        add_way_point(points...);       
    }






    Way(data_type radius,std::initializer_list<Point<data_type>> points):Way(radius)
    {

        if(points.size()<3) throw "no enough waypoints";
        for(auto point:points)
        {
            way_points.push_back(point);
        }
        
    }
    ~Way()
    {
        way_points.clear();
    }

    //functions to add_way_points
    template<typename T>
    void add_way_point(
        const Point<T>& point
        )
    {
        
        way_points.push_back(point);
    }

    template<typename T, typename ... Ts>
    void add_way_point(
        const Point<T>& point,
        Ts ... points
        )
    {
        
        way_points.push_back(point);
        add_way_point(points...);
    }


    virtual data_type GetChi_c(
        const DerivedPlantData<data_type>& derivedData, 
        const Point<data_type>& location,
        const data_type* Vb
        ) override
    {
        std::unique_ptr<Path<data_type>> mission;
        mission = this->GetMission(location);

        return mission->GetChi_c(derivedData,location,Vb);
    }



    virtual std::unique_ptr<Path<data_type>> GetMission(
        const Point<data_type>& location
        ) override
    {
        //std::cout<<"radius:"<<radius<<std::endl;
        //q_i_min_1 = (w(vF_Parameters.traversed_waypoint_count,:)-w(vF_Parameters.traversed_waypoint_count-1,:)) / norm(w(vF_Parameters.traversed_waypoint_count,:)-w(vF_Parameters.traversed_waypoint_count-1,:),2);
		auto q_i_min_1 = (way_points[1]-way_points[0]).unit();

        if(way_points.size()>2){
            
            //q_i = (w(vF_Parameters.traversed_waypoint_count+1,:)-w(vF_Parameters.traversed_waypoint_count,:)) / norm(w(vF_Parameters.traversed_waypoint_count+1,:)-w(vF_Parameters.traversed_waypoint_count,:),2);
            auto q_i = (way_points[2]-way_points[1]).unit();
            //rho = acos(-q_i_min_1*q_i');
            data_type rho = std::acos(-q_i_min_1.dot(q_i));

        
 		
			if(linemode)
			{	
                if(lineMission == nullptr)
                    lineMission = new Line<data_type>();
                //vF_Parameters.line_origin = w(vF_Parameters.traversed_waypoint_count-1,:);
                

                Line<data_type> *mission = new Line<data_type>();
                
				mission->path_origin = way_points[0];
				// /vF_Parameters.line_slope = q_i_min_1;
                
				mission->slope = q_i_min_1;
                //std::cout<<"radius:"<<radius;
				//vF_Parameters.z = w(vF_Parameters.traversed_waypoint_count,:) - (vF_Parameters.orbit_radius/(tan(rho/2)))*q_i_min_1;
				corner= (way_points[1]-(q_i_min_1*radius)/std::tan(rho/2));
                //std::cout<<"corner"<<corner.x<<","<<corner.x<<std::endl;
                /*if (p-vF_Parameters.z) * q_i_min_1' >= 0
	        		state = 2;
	        		% vF_Parameters.traversed_waypoint_count=vF_Parameters.traversed_waypoint_count+1;
	   				end*/
				if(((location-corner).dot(q_i_min_1))>=0)
                {
					linemode=false;
                    if(lineMission!=nullptr){
                        delete lineMission;
                        lineMission = nullptr;
                    }
                }

                return std::unique_ptr<Path<data_type>>(mission);
			}
			else
			{
                if(orbitMission == nullptr)
                {
                    orbitMission = new Orbit<data_type>();
                    orbitMission->radius = radius;
                }
                
                Orbit<data_type> *mission = new Orbit<data_type>();
                mission->radius = radius;
                //std::cout<<"switchedlocation"<<location.x<<","<<location.y<<std::endl;
                //exit(0);
				//vF_Parameters.orbit_centre = w(vF_Parameters.traversed_waypoint_count,:)-(vF_Parameters.orbit_radius/(sin(rho/2)))*(q_i_min_1 - q_i)/(norm(q_i_min_1- q_i));
				mission->centre=(way_points[1]-((q_i_min_1-q_i).unit()*radius/std::sin(rho/2)));
                
	    		//vF_Parameters.orbit_lambda = sign(q_i_min_1(1) * q_i(2) - q_i_min_1(2)*q_i(1));
	    		float value = (q_i_min_1.x * q_i.y - q_i_min_1.y*q_i.x);
	    		mission->direction = (value==0)?0:((value<0)?-1:1);

	    			//vF_Parameters.z = w(vF_Parameters.traversed_waypoint_count,:) + (vF_Parameters.orbit_radius/(tan(rho/2)))*q_i;
	    			corner=(way_points[1] + (q_i*radius/(tan(rho/2))));
	    			//if (p-vF_Parameters.z) * q_i' >= 0
	    			if((location-corner).dot(q_i)>=0)
	    			{

	        		//vF_Parameters.traversed_waypoint_count = vF_Parameters.traversed_waypoint_count + 1;
	        		//state = 1;

	    				linemode = true;
                        if(orbitMission!= nullptr){
                            delete orbitMission;
                            orbitMission = nullptr;
                        }
                        way_points.erase(way_points.begin());
	    			}  
                return std::unique_ptr<Path<data_type>>(mission);
			}
        }
        else
        {
            Line<data_type> *mission = new Line<data_type>();

			mission->path_origin = way_points[0];
			// /vF_Parameters.line_slope = q_i_min_1;
			mission->slope = q_i_min_1;
            return std::unique_ptr<Path<data_type>>(mission);
        }
		
    }


    private:
    bool linemode = true;
    Point<data_type> corner;
    data_type radius;
    Line<data_type> *lineMission = nullptr;
    Orbit<data_type> *orbitMission = nullptr;

};


#endif
