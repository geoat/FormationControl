#pragma once
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <set>
#include <exception>
#include <iostream>
#include "../Types/formationtypes.hpp"
#include <map>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <cstdlib>
#include <string>
#include <exception>

#include "pathplannerdata.hpp"
#include "pathtype.hpp"
#include "point.hpp"

#define CONFIGURATION_PATH "formation.config"
namespace pt = boost::property_tree;

template<typename data_type>
struct Configuration
{
    typedef Eigen::SparseMatrix<data_type> SpMat;
    typedef Eigen::Triplet<data_type> Triplet;
    //Key: DroneID, Value<drone type, index>
    std::map < _droneID_type, std::pair<unsigned int, unsigned int>> dronesAndTypes;
    SpMat communications;
    std::vector<Configuration::Triplet> communicationEntries;
    data_type KpGain[6];
    data_type KvGain[6];
    SpMat S;
    PathPlannerData<data_type> pathPlannerData;
    std::vector<Configuration::Triplet> SEntries;
    SpMat P;
    std::vector<Configuration::Triplet> PEntries;
    void load(const std::string &filename);
    void save(const std::string &filename);
    private:
    Point<data_type> GetPointFromString(const std::string &str);
};
template <typename data_type>
void Configuration<data_type>::load(const std::string &filename)
{
    try{
        dronesAndTypes.clear();
        // Create empty property tree object
        pt::ptree tree;

        // Parse the XML into the property tree.
        pt::read_xml(filename, tree);

        /* configure path details */
        pathPlannerData.pathType = (PathType)tree.get<unsigned int>("configuration.path.pathtype");
        //std::cout << (int)pathPlannerData.pathType << std::endl;
        
        //based on path type process further path specifics
        switch (pathPlannerData.pathType)
        {
            case PathType::Line:{
                std::string str(tree.get<std::string>("configuration.path.points.lineorigin"));
                pathPlannerData.lineOrigin = GetPointFromString(str);
                str = tree.get<std::string>("configuration.path.points.lineslope");
                pathPlannerData.lineSlope = GetPointFromString(str);
            }

                break;
            case PathType::Orbit:
            {
                pathPlannerData.radius = (data_type)tree.get("configuration.path.radius", 50.00);
                std::string str(tree.get<std::string>("configuration.path.points.orbitcentre"));
                pathPlannerData.orbitCentre = GetPointFromString(str);
                //std::cout << pathPlannerData.orbitCentre.x << "x,y" << pathPlannerData.orbitCentre.y<< std::endl;
            }
            break;
            case PathType::WayPoints:
            {
                pathPlannerData.radius = (data_type)tree.get("configuration.path.radius", 75.00);
                //std::cout<<pathPlannerData.radius<<std::endl;
                pathPlannerData.wayPoints.clear();
                int wayPointsCount = 0;
                BOOST_FOREACH (pt::ptree::value_type &v, tree.get_child("configuration.path.points"))
                {
                    if(v.first.compare("waypoint")==0){
                        wayPointsCount++;
                        std::string wayPointStr = v.second.data();
                        pathPlannerData.wayPoints.push_back(GetPointFromString(wayPointStr));
                        // std::pair<unsigned int, unsigned int> data;
                        // data.first = v.second.get<unsigned int>("dronetype");//drone type
                        // data.second = dronesAndTypes.size();//index
                        // dronesAndTypes.emplace(droneID, data);
                    }
                }
                //std::cout<<pathPlannerData.wayPoints.size()<<std::endl;
                if(wayPointsCount<3)
                {
                    throw "need minimum three points";
                }
            }
            break;
            default:
            break;
        }
        


        /*iterate through all drones*/
        // Use get_child to find the node containing the modules, and iterate over
        // its children. If the path cannot be resolved, get_child throws.
        // A C++11 for-range loop would also work.
        BOOST_FOREACH (pt::ptree::value_type &v, tree.get_child("configuration.drones"))
        {
            // The data function is used to access the data stored in a node.
            _droneID_type droneID = v.second.get<_droneID_type>("droneid");
            std::pair<unsigned int, unsigned int> data;
            data.first = v.second.get<unsigned int>("dronetype");//drone type
            data.second = dronesAndTypes.size();//index
            dronesAndTypes.emplace(droneID, data);
        }

        communications.resize((int)dronesAndTypes.size(), (int)dronesAndTypes.size());
        communicationEntries.clear();
            /*iterate through all communications*/
            // Use get_child to find the node containing the modules, and iterate over
            // its children. If the path cannot be resolved, get_child throws.
            // A C++11 for-range loop would also work.
        BOOST_FOREACH (pt::ptree::value_type &v, tree.get_child("configuration.communications"))
        {
            // The data function is used to access the data stored in a node.
            _droneID_type sourceDrone = v.second.get<_droneID_type>("sourcedrone");
            _droneID_type targetDrone = v.second.get<unsigned int>("targetdrone");
            unsigned int sourceDroneIndex;
            unsigned int targetDroneIndex;

            auto search = dronesAndTypes.find(sourceDrone);
            if (search != dronesAndTypes.end())
            {
                sourceDroneIndex = search->second.second;
            }
            else
            {
                throw "communication with wrong source drone id";
            }

            search = dronesAndTypes.find(targetDrone);
            if (search != dronesAndTypes.end())
            {
                targetDroneIndex = search->second.second;
            }
            else
            {
                throw "communication with wrong target drone id";
            }
            communicationEntries.push_back(Triplet(sourceDroneIndex, targetDroneIndex, 1));
            //dronesAndTypes.emplace(droneID, dronetype);
        }
        communications.setFromTriplets(communicationEntries.begin(), communicationEntries.end());

        /*Kp Values*/
        int i = 0;
        BOOST_FOREACH (pt::ptree::value_type &v, tree.get_child("configuration.kpvalues"))
        {
            KpGain[i] = atof(v.second.data().c_str());
            i++;
        }
        /*Kv Values*/
        i = 0;
        BOOST_FOREACH (pt::ptree::value_type &v, tree.get_child("configuration.kvvalues"))
        {
            KvGain[i] = atof(v.second.data().c_str());
            i++;
        }
        
        /*S Gain*/
        S.resize(6,6);
        SEntries.clear();
        BOOST_FOREACH (pt::ptree::value_type &v, tree.get_child("configuration.sgain"))
        {
            unsigned int i = v.second.get<unsigned int>("<xmlattr>.row");
            unsigned int j = v.second.get<unsigned int>("<xmlattr>.column");

            if((i<6) & (j <6))
            {
                SEntries.push_back(Triplet(i, j, v.second.get<data_type>("")));
            }
        }
        S.setFromTriplets(SEntries.begin(), SEntries.end());


        /*P Gain*/
        P.resize(12, 12);
        PEntries.clear();
        BOOST_FOREACH (pt::ptree::value_type &v, tree.get_child("configuration.pgain"))
        {
            unsigned int i = v.second.get<unsigned int>("<xmlattr>.row");
            unsigned int j = v.second.get<unsigned int>("<xmlattr>.column");

            if ((i < 12) & (j < 12))
            {
                PEntries.push_back(Triplet(i, j, v.second.get<data_type>("")));
            }
        }
        P.setFromTriplets(PEntries.begin(), PEntries.end());
    }
    catch(const char* msg)
    {
        std::cout<<"Configuration parsing failed"<<std::endl;
        std::cout << msg << std::endl;
        exit(0);
    }
    catch(std::exception& e){
        std::cout << "Configuration parsing failed" << std::endl;
        std::cout << e.what() << std::endl;
        exit(0);
    }
}

template <typename data_type>
void Configuration<data_type>::save(const std::string &filename)
{
    // Create an empty property tree object.
    pt::ptree tree;


    // Add all the modules. Unlike put, which overwrites existing nodes, add
    // adds a new node at the lowest level, so the "modules" node will have
    // multiple "module" children.
    boost::property_tree::ptree drones;
    BOOST_FOREACH (const auto &data, dronesAndTypes){
        boost::property_tree::ptree drone;
        drone.add("droneid", data.first);
        drone.add("dronetype", data.second.first);
        drones.add_child("drone", drone);
    }
    tree.add_child("configuration.drones",drones);

    // Add all the modules. Unlike put, which overwrites existing nodes, add
    // adds a new node at the lowest level, so the "modules" node will have
    // multiple "module" children.
    boost::property_tree::ptree communications;
    BOOST_FOREACH (const auto &data, communicationEntries)
    {
        boost::property_tree::ptree communication;
        communication.add("sourcedrone", data.row());
        communication.add("targetdrone", data.col());
        communications.add_child("communication", communication);
    }
    tree.add_child("configuration.communications", communications);

    boost::property_tree::ptree kpvalues;
    for(int i = 0;i<6;i++)
    {
        boost::property_tree::ptree element;
        element.add("<xmlattr>.index", std::to_string(i));
        element.put_value(std::to_string(KpGain[i]));
        kpvalues.add_child("element", element);
    }
    tree.add_child("configuration.kpvalues", kpvalues);

    boost::property_tree::ptree kvvalues;
    for (int i = 0; i < 6; i++)
    {
        boost::property_tree::ptree element;
        element.add("<xmlattr>.index", std::to_string(i));
        element.put_value(std::to_string(KvGain[i]));
        kvvalues.add_child("element", element);
    }
    tree.add_child("configuration.kvvalues", kvvalues);
    boost::property_tree::ptree sgain;
    BOOST_FOREACH (const auto& entry,SEntries){
            boost::property_tree::ptree element;
            element.add("<xmlattr>.row", entry.row());
            element.add("<xmlattr>.column", entry.col());
            element.put_value(std::to_string(entry.value()));
            sgain.add_child("element", element);

    }
    tree.add_child("configuration.sgain", sgain);

    boost::property_tree::ptree pgain;
    BOOST_FOREACH (const auto &entry, PEntries)
    {
        boost::property_tree::ptree element;
        element.add("<xmlattr>.row", entry.row());
        element.add("<xmlattr>.column", entry.col());
        element.put_value(std::to_string(entry.value()));
        pgain.add_child("element", element);
    }
    tree.add_child("configuration.pgain", pgain);

    // Write property tree to XML file
    //pt::write_xml(filename, tree);
}

struct debug_settings
{
    std::string m_file;              // log filename
    int m_level;                     // debug level
    std::set<std::string> m_modules; // modules where logging is enabled
    void load(const std::string &filename);
    void save(const std::string &filename);
};

void debug_settings::load(const std::string &filename)
{
    // Create empty property tree object
    pt::ptree tree;

    // Parse the XML into the property tree.
    pt::read_xml(filename, tree);

    // Use the throwing version of get to find the debug filename.
    // If the path cannot be resolved, an exception is thrown.
    m_file = tree.get<std::string>("debug.filename");

    // Use the default-value version of get to find the debug level.
    // Note that the default value is used to deduce the target type.
    m_level = tree.get("debug.level", 0);

    // Use get_child to find the node containing the modules, and iterate over
    // its children. If the path cannot be resolved, get_child throws.
    // A C++11 for-range loop would also work.
    BOOST_FOREACH (pt::ptree::value_type &v, tree.get_child("debug.modules"))
    {
        // The data function is used to access the data stored in a node.
        m_modules.insert(v.second.data());
    }
}

void debug_settings::save(const std::string &filename)
{
    // Create an empty property tree object.
    pt::ptree tree;

    // Put the simple values into the tree. The integer is automatically
    // converted to a string. Note that the "debug" node is automatically
    // created if it doesn't exist.
    tree.put("debug.filename", m_file);
    tree.put("debug.level", m_level);

    // Add all the modules. Unlike put, which overwrites existing nodes, add
    // adds a new node at the lowest level, so the "modules" node will have
    // multiple "module" children.
    BOOST_FOREACH (const std::string &name, m_modules)
        tree.add("debug.modules.module", name);

    // Write property tree to XML file
    pt::write_xml(filename, tree);
}
template<typename data_type>
Point<data_type> Configuration<data_type>::GetPointFromString(const std::string &str)
{   
    data_type x,y;
    try{
    std::vector<std::string> vec;
    boost::algorithm::split(vec, str, boost::is_any_of(":,;"));
        if(vec.size()>=2)
        {
            x = atof(vec[0].c_str());
            y = atof(vec[1].c_str());
        }
        else
            throw "no enough acceptable dimensional components";
    }
    catch(const char * msg)
    {
        std::cout << msg << std::endl;
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
        std::cout << "Error in parsing configurations" << std::endl;
            exit(0);
    }
    return Point<data_type>(x,y);

}