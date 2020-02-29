#ifndef NODETYPE_HPP
#define NODETYPE_HPP

#include <vector>
#include <cstring>
#include <algorithm>
#include <stdio.h>

#include "Data/formationdata.hpp"
#include "Environment/windmodel.hpp"
#include "PathPlanner/pathmanager.hpp"

//#include "Leader/leadercontrol_test.hpp"

#include "Data/dronedata.hpp"

#include <common/mavlink.h>
#include "PX4/autopilot_interface.h"
#include <fstream>

#ifndef HITL
#include "DroneSim/dronemodel.hpp"
#endif
#include "PathPlanner/dronewithidc.hpp"

#include "Timing/timing.hpp"

#include "Maths/math_helper.hpp"

#include "Leader/leadercontrol.hpp"
#include "Follower/followercontrol.hpp"

#include "../Data/controloutput.hpp"

#include "../Data/nodetype.hpp"
template <typename data_type = double>
class Node
{

public:
    data_type samplingTime;
    data_type controlTime;
    NodeType nodeType;

    volatile bool execute = false;
    volatile bool executeControl = false;

    virtual void InitializeType() {}
#ifdef HITL
    virtual void Execute(
        FormationData<data_type> &formationData,
        Autopilot_Interface &api)
    {
    }
#else
    virtual void Execute(
        FormationData<data_type> &formationData)
    {
    }

    virtual void FillPlantData(DroneData<data_type> &droneData)
    {
    }

#endif
    virtual void ReleaseType()
    {
    }

    inline void HandleTimerInterrupt()
    {

        execute = true;
        timeAfterLastControl += samplingTime;
        if (timeAfterLastControl >= controlTime)
        {

            executeControl = true;
            timeAfterLastControl = 0;
        }
    }

    Node() : nodeType(NodeType::Basic)
    {
    }

    Node(
        data_type samplingTime,
        data_type controlTime) : samplingTime(samplingTime),
                                 controlTime(controlTime), nodeType(NodeType::Basic)
    {
    }
    virtual ~Node() {}

private:
    float timeAfterLastControl = 0;
};

template <typename data_type = double>
class PathPlanner : public Node<data_type>
{
    std::ofstream pathplannerStream;
    std::ofstream virtualDroneStream;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Node<data_type> BasicNode;
    typedef Eigen::Matrix<data_type, 6, 1> q_type;

    PathManager<data_type> pathmanager;
    data_type chi_c;
    q_type referenceInput;
#ifndef HITL
    Drone<data_type, true, true> drone;
    WindModel<data_type, true> windModel;
    data_type VT = 15;
    data_type HT = 50;
#else
    WindModel<data_type, false> windModel;
#endif

    DroneWithIDC<data_type> droneWithIDC;

    PathPlanner(
        data_type samplingTime,
        data_type controlTime) : pathmanager(controlTime)
    {
        BasicNode::samplingTime = samplingTime;
        BasicNode::controlTime = controlTime;
        BasicNode::nodeType = NodeType::PathPlanner;
        InitializeType();
        pathplannerStream.open("pathplanner_output.Csv");
        virtualDroneStream.open("virtualdrone_output.Csv");
    }

    virtual void InitializeType() override
    {
        //declaring matrix and vector types used
        typedef Eigen::Matrix<data_type, 3, 1> vector_type;
        typedef Eigen::Matrix<data_type, 3, 3> matrix_type;

        data_type Mass = 1;
        matrix_type I;
        I << 0.02, 0, -0.01,
            0, 0.026, 0,
            -0.01, 0, 0.053;

        //setting intial conditions for states
        //xyz initial
        //xyz = [0,500,-50]';

        vector_type Xe_init;
        Xe_init << -100, 100, -50;

        vector_type Xe_init2;
        Xe_init2 = Xe_init;

        //uvw initial
        //uvw = [15,0,0]';
        vector_type Vb_init;
        Vb_init << 15, 0, 0;

        //euler initial
        //euler = [0.1;0.1;1];
        vector_type Euler_init;
        Euler_init << 0.1, 0.1, 1;

        vector_type Euler_init2;
        Euler_init2 << 0.1, 0.1, 1;

        //pqr initial
        //pqr = [1;1;2];
        vector_type pqr_init;
        pqr_init << 1, 1, 2;

        vector_type pqr_init2;
        pqr_init2 << 1, 1, 2;

#ifndef HITL
        drone = Drone<data_type, true, true>(BasicNode::samplingTime, Mass, Xe_init, Vb_init, Euler_init, pqr_init);
#endif

        data_type Mass2 = 1.0;
        droneWithIDC = DroneWithIDC<data_type>(BasicNode::samplingTime, Mass2, I, Xe_init2, Vb_init, Euler_init2, pqr_init2);
        auto &formationData = FormationData<data_type>::GetFormationData();
        if (!formationData.IsNewPathPlannerDataAvailable())
        {
            // defining default path
            auto point1 = Point<data_type>(0, 0);
            auto point2 = Point<data_type>(500, 500);
            auto point3 = Point<data_type>(500, 0);
            auto point4 = Point<data_type>(0, 500);
            auto point5 = Point<data_type>(0, 0);
            pathmanager.SetPathAsWay(30, point1, point2, point3, point4, point5, point2, point3);
        }
        else
        {
            std::cout << "setting received path" << std::endl;
            pathmanager.set_path(formationData.GetPathPlannerData());
            formationData.ClearNewPathPlannerDataAvailableFlag();
        }
    }

#ifdef HITL
    virtual void Execute(
        FormationData<data_type> &formationData,
        Autopilot_Interface &api) override
    {

        BasicNode::execute = false;
        auto &droneStates = formationData.GetCurrentDroneData().droneStates;
        DerivedPlantData<data_type> derivedDroneData(formationData.GetCurrentDroneData().droneStates, windModel.ConstantComponent.Value);
        currentLocation.x = droneStates[0];
        currentLocation.y = droneStates[1];

        chi_c = pathmanager.path->GetChi_c(derivedDroneData, currentLocation, &(droneStates[3]));

        if (BasicNode::executeControl)
        {
            BasicNode::executeControl = false;

            //Execute Control action
            mavlink_set_position_target_local_ned_t sp;

            sp.type_mask = MAVLINK_MSG_MASK_ALL;

            //send control command
            set_yaw(chi_c, // [rad]
                    sp);

            // SEND THE COMMAND
            api.update_setpoint(sp);
        }
    }
#else

    virtual void Execute(
        FormationData<data_type> &formationData) override
    {
        static int count = 0;
        BasicNode::execute = false;

        //copying current location
        currentLocation.x = drone.y[0];
        currentLocation.y = drone.y[1];

        //initializing windModel
        auto windData = windModel.GetWindData(drone.t);

        //generating derived data with total components of wind - ONLY for simulation
        DerivedPlantData<data_type> derivedData(drone.y, windData.TotalWind.Value);

        //generating derived data with only constant known components of wind
        DerivedPlantData<data_type> derivedData_course(drone.y, windData.ConstantComponent.Value);

        if (BasicNode::executeControl)
        {
            BasicNode::executeControl = false;
            //count = 0;
            //std::cout << GetCurrentTime_ms()<<std::endl;

            //calculating course angle input
            chi_c = pathmanager.path->GetChi_c(derivedData_course, currentLocation, &(drone.y[3]));
        }

        //helper code for writing data to flat files
        //std::cout << currentLocation.x << "," << currentLocation.y << std::endl; //<<","<<chi_c*180/M_PI<<std::endl;
        pathplannerStream << currentLocation.x << "," << currentLocation.y << std::endl;
        //std::cout<<location2.x<<","<<location2.y<<std::endl;//<<","<<chi_c*180/M_PI<<std::endl;
        virtualDroneStream << droneWithIDC.y[0] << "," << droneWithIDC.y[1] << std::endl;

        //calculating the reference input r
        referenceInput = ReferenceModel<data_type>::GetReferenceModel().CalculateReferenceInput(drone.y, derivedData, drone.acceleration, drone.wbdot);

        //simulating drone dynamics
        drone.DoNextStep(chi_c * 180 / M_PI, VT, HT);

        //running virtual drone based calculation
        droneWithIDC.DoNextStep(referenceInput, ReferenceModel<data_type>::GetReferenceModel());
    }

    virtual void FillPlantData(DroneData<data_type> &droneData) override
    {
        //updating droneStates as data of virtual drone (reference drone)
        //note: rotation of Xe should be handled in data consumer side
        droneData.droneStates.clear();
        droneData.droneStates = droneWithIDC.y;

        //updating control input as Reference input calculated
        droneData.controlInput.clear();
        droneData.controlInput.insert(droneData.controlInput.begin(), referenceInput.data(), referenceInput.data() + 6);
    }

#endif

    virtual void ReleaseType() override
    {
    }

    ~PathPlanner() override
    {
        pathplannerStream.close();
        virtualDroneStream.close();
    }

private:
    Point<data_type> currentLocation;
};

template <typename data_type = double>
class LeaderNode : public Node<data_type>
{

#ifndef HITL
    Drone<data_type, false, false>
        drone;

#else

#endif

public:
    typedef Node<data_type> BasicNode;
    typedef Eigen::Matrix<data_type, 3, 1> vector_type;

    LeaderNode(
        FormationData<data_type> &formationData,
        data_type samplingTime,
        data_type controlTime) : leaderController(samplingTime)

    {
        BasicNode::samplingTime = samplingTime;
        BasicNode::controlTime = controlTime;
        BasicNode::nodeType = NodeType::Leader;
        InitializeType();
    }

    virtual void InitializeType() override
    {
#ifndef HITL
        //declaring matrix and vector types used

        typedef Eigen::Matrix<data_type, 3, 3> matrix_type;

        data_type Mass = 1;
        matrix_type I;
        I << 0.02, 0, -0.01,
            0, 0.026, 0,
            -0.01, 0, 0.053;

        //setting intial conditions for states
        //xyz initial
        //xyz = [0,500,-50]';

        vector_type Xe_init;
        Xe_init << 500, 500, -50;


        //uvw initial
        //uvw = [15,0,0]';
        vector_type Vb_init;
        Vb_init << 15, 0, 0;

        //euler initial
        //euler = [0.1;0.1;1];
        vector_type Euler_init;
        Euler_init << 0.1, 0.1, 1;



        //pqr initial
        //pqr = [1;1;2];
        vector_type pqr_init;
        pqr_init << 1, 1, 2;



        drone = Drone<data_type, true, true>(BasicNode::samplingTime, Mass, Xe_init, Vb_init, Euler_init, pqr_init);
#endif
    }

#ifdef HITL
    virtual void Execute(
        FormationData<data_type> &formationData,
        Autopilot_Interface &api) override
    {

        BasicNode::execute = false;
        //Calculations

        if (BasicNode::executeControl)
        {
            BasicNode::executeControl = false;

            //Execute Control action
            mavlink_set_position_target_local_ned_t sp;

            sp.type_mask = MAVLINK_MSG_MASK_ALL;

            // SEND THE COMMAND
            api.update_setpoint(sp);
        }
    }
#else

    virtual void Execute(
        FormationData<data_type> &formationData) override
    {

        BasicNode::execute = false;
        bool validControl = false;

        //testLeaderEstimator();

        //calculate leader controller output
        validControl = leaderController.CalculateFandM(controlOutput);

        if (!validControl)
        {

            return;
        }

        if (BasicNode::executeControl)
        {
            BasicNode::executeControl = false;

            //Execute Control action
        }

        //Simulation
        data_type netF_arr[3];
        std::copy(std::begin(controlOutput.F), std::end(controlOutput.F), netF_arr);
        //add gravitational action
        DerivedPlantData<data_type> droneDerivedData(drone.y);
        vector_type ge;
        ge << 0,
            0,
            drone.Mass * Constants_g;
        auto gb = droneDerivedData.DCMeb.transpose() * ge;
        Eigen::Map<vector_type> netF(netF_arr);
        netF = netF - gb;
        //integrate
        drone.DoNextStep(netF_arr, controlOutput.M);

        //printf("calculation reached\n");
    }

    virtual void FillPlantData(DroneData<data_type> &droneData) override
    {
        droneData.droneStates = drone.y;
        //RotateFromEarthToBody(droneData.droneStates.data(),droneData.droneStates.data()+6);
        //std::cout << "droneData.controlInput[12]:" << droneData.controlInput[12] << std::endl;
        data_type *controlInput_arr = droneData.controlInput.data();
        std::copy(controlOutput.F, controlOutput.F + 3, controlInput_arr);
        std::copy(controlOutput.M, controlOutput.M + 3, controlInput_arr + 3);
    }

#endif

    virtual void ReleaseType() override
    {
    }

    ~LeaderNode() override {}

private:
    Point<data_type> currentLocation;
    LeaderController<data_type> leaderController;
    ControlOutput<data_type> controlOutput;
};

template <typename data_type = double>
class FollowerNode : public Node<data_type>
{

#ifndef HITL
    Drone<data_type, false, false>
        drone;

#else

#endif

public:
    typedef Node<data_type> BasicNode;
    typedef Eigen::Matrix<data_type, 3, 1> vector_type;

    FollowerNode(
        FormationData<data_type> &formationData,
        data_type samplingTime,
        data_type controlTime) : followerController(samplingTime)

    {
        BasicNode::samplingTime = samplingTime;
        BasicNode::controlTime = controlTime;
        BasicNode::nodeType = NodeType::Follower;
        InitializeType();
    }

    virtual void InitializeType() override
    {
#ifndef HITL
        //declaring matrix and vector types used

        typedef Eigen::Matrix<data_type, 3, 3> matrix_type;

        data_type Mass = 1;
        matrix_type I;
        I << 0.02, 0, -0.01,
            0, 0.026, 0,
            -0.01, 0, 0.053;

        //setting intial conditions for states
        //xyz initial
        //xyz = [0,500,-50]';

        vector_type Xe_init;
        Xe_init << 0, 500, -50;



        //uvw initial
        //uvw = [15,0,0]';
        vector_type Vb_init;
        Vb_init << 15, 0, 0;

        //euler initial
        //euler = [0.1;0.1;1];
        vector_type Euler_init;
        Euler_init << 0.1, 0.1, 1;



        //pqr initial
        //pqr = [1;1;2];
        vector_type pqr_init;
        pqr_init << 1, 1, 2;



        drone = Drone<data_type, true, true>(BasicNode::samplingTime, Mass, Xe_init, Vb_init, Euler_init, pqr_init);
#endif
    }

#ifdef HITL
    virtual void Execute(
        FormationData<data_type> &formationData,
        Autopilot_Interface &api) override
    {

        BasicNode::execute = false;
        //Calculations

        if (BasicNode::executeControl)
        {
            BasicNode::executeControl = false;

            //Execute Control action
            mavlink_set_position_target_local_ned_t sp;

            sp.type_mask = MAVLINK_MSG_MASK_ALL;

            // SEND THE COMMAND
            api.update_setpoint(sp);
        }
    }
#else

    virtual void Execute(
        FormationData<data_type> &formationData) override
    {

        BasicNode::execute = false;
        bool validControl = false;

        //testFollowerEstimator();

        //calculate follower controller output
        validControl = followerController.CalculateFandM(controlOutput);

        if (!validControl)
        {

            return;
        }

        if (BasicNode::executeControl)
        {
            BasicNode::executeControl = false;

            //Execute Control action
        }

        //Simulation
        data_type netF_arr[3];
        std::copy(std::begin(controlOutput.F), std::end(controlOutput.F), netF_arr);
        //add gravitational action
        DerivedPlantData<data_type> droneDerivedData(drone.y);
        vector_type ge;
        ge << 0,
            0,
            drone.Mass * Constants_g;
        auto gb = droneDerivedData.DCMeb.transpose() * ge;
        Eigen::Map<vector_type> netF(netF_arr);
        netF = netF - gb;
        //integrate
        drone.DoNextStep(netF_arr, controlOutput.M);

        //printf("calculation reached\n");
    }

    virtual void FillPlantData(DroneData<data_type> &droneData) override
    {
        droneData.droneStates = drone.y;
        //RotateFromEarthToBody(droneData.droneStates.data(),droneData.droneStates.data()+6);
        //std::cout << "droneData.controlInput[12]:" << droneData.controlInput[12] << std::endl;
        data_type *controlInput_arr = droneData.controlInput.data();
        std::copy(controlOutput.F, controlOutput.F + 3, controlInput_arr);
        std::copy(controlOutput.M, controlOutput.M + 3, controlInput_arr + 3);
    }

#endif

    virtual void ReleaseType() override
    {
    }

    ~FollowerNode() override {}

private:
    Point<data_type> currentLocation;
    FollowerController<data_type> followerController;
    ControlOutput<data_type> controlOutput;
};

#endif
