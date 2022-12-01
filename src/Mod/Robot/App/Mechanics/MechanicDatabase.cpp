// Created By Yixiao 2022-10-13

#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_

#endif

#include <stdio.h>
#include <iostream>
#include <algorithm>

#include <Base/Console.h>
#include <Base/VectorPy.h>

#include <App/Application.h>
#include "MechanicDatabase.h"


using namespace Robot;
using namespace std;

MechanicDatabase::MechanicDatabase(void)
{
    append_RobotData();
    append_PoserData();
    append_ExtAxData();
}

MechanicDatabase::~MechanicDatabase()
{

}

const RobotModelInfo MechanicDatabase::getTargetRobotItemInfo(const string t_brandName, const string t_ModelName)
{
    RobotModelInfo t_robotModelInfo = RobotModelInfo("", RobotType::_Standard,0,0,"");
    RobotBrand t_Brand;
    for(uint i = 0; i < m_Database_Robot.size(); i++){
        if(std::string(RobotBrand_str[i]) == t_brandName)
            t_Brand = (RobotBrand)i;
    }
    auto result = m_Database_Robot.find(t_Brand);
    if(result==m_Database_Robot.end())
        return t_robotModelInfo;
    for(auto item : result->second){
        if(item.model_Name == t_ModelName)
            t_robotModelInfo = item;
    }
    return t_robotModelInfo;
}

const std::vector<RobotBrand> MechanicDatabase::getAvailableRobotBrands() const
{
    std::vector<RobotBrand> brands;
    for(auto iter = m_Database_Robot.begin(); iter!=m_Database_Robot.end();iter++){
        if(std::find_if(brands.begin(), brands.end(), [&](const RobotBrand& t_brand){
                        return iter->first == t_brand;}) == brands.end()){
            brands.push_back(iter->first);
        }
    }
    return brands;
}

const std::vector<PoserBrand> MechanicDatabase::getAvailablePoserBrands() const
{
    std::vector<PoserBrand> brands;
    for(auto iter = m_Database_Poser.begin(); iter!=m_Database_Poser.end();iter++){
        if(std::find_if(brands.begin(), brands.end(), [&](const PoserBrand& t_brand){
                        return iter->first == t_brand;}) == brands.end()){
            brands.push_back(iter->first);
        }
    }
    return brands;
}

const std::vector<ExtAxBrand> MechanicDatabase::getAvailableExtAxBrands() const
{
    std::vector<ExtAxBrand> brands;
    for(auto iter = m_Database_ExtAx.begin(); iter!=m_Database_ExtAx.end();iter++){
        if(std::find_if(brands.begin(), brands.end(), [&](const ExtAxBrand& t_brand){
                        return iter->first == t_brand;}) == brands.end()){
            brands.push_back(iter->first);
        }
    }
    return brands;
}

const std::vector<RobotType> MechanicDatabase::getAvailableRobotType(const RobotBrand &t_Brand)
{
    std::vector<RobotType> result;
    auto iter = m_Database_Robot.find(t_Brand);
    if(iter != m_Database_Robot.end()){
        for(auto item : iter->second){
            if(std::find_if(result.begin(), result.end(), [&](const RobotType& t_Type){
                            return t_Type == item.model_Type;}) == result.end()){
                result.push_back(item.model_Type);
            }
        }
    }
    return result;
}

const std::vector<PoserType> MechanicDatabase::getAvailablePoserType(const PoserBrand &t_Brand)
{
    std::vector<PoserType> result;
    auto iter = m_Database_Poser.find(t_Brand);
    if(iter != m_Database_Poser.end()){
        for(auto item : iter->second){
            if(std::find_if(result.begin(), result.end(), [&](const PoserType& t_Type){
                            return t_Type == item.model_Type;}) == result.end()){
                result.push_back(item.model_Type);
            }
        }
    }
    return result;
}

const std::vector<ExtAxType> MechanicDatabase::getAvailableExtAxType(const ExtAxBrand &t_Brand)
{
    std::vector<ExtAxType> result;
    auto iter = m_Database_ExtAx.find(t_Brand);
    if(iter != m_Database_ExtAx.end()){
        for(auto item : iter->second){
            if(std::find_if(result.begin(), result.end(), [&](const ExtAxType& t_Type){
                            return t_Type == item.model_Type;}) == result.end()){
                result.push_back(item.model_Type);
            }
        }
    }
    return result;
}

const std::vector<uint> MechanicDatabase::getAvailableRobot_DOF(const RobotBrand &t_Brand, const RobotType &t_Type)
{
    std::vector<uint> result;
    auto iter = m_Database_Robot.find(t_Brand);
    if(iter != m_Database_Robot.end()){
        for(auto item : iter->second){
            if(item.model_Type == t_Type){
                if(std::find(result.begin(), result.end(), item.model_dof) == result.end()){
                    result.push_back(item.model_dof);
                }
            }
        }
    }
    return result;
}

const std::vector<uint> MechanicDatabase::getAvailablePoser_DOF(const PoserBrand &t_Brand, const PoserType &t_Type)
{
    std::vector<uint> result;
    auto iter = m_Database_Poser.find(t_Brand);
    if(iter != m_Database_Poser.end()){
        for(auto item : iter->second){
            if(item.model_Type == t_Type){
                if(std::find(result.begin(), result.end(), item.model_dof) == result.end()){
                    result.push_back(item.model_dof);
                }
            }
        }
    }
    return result;
}

const std::vector<uint> MechanicDatabase::getAvailableExtAx_DOF(const ExtAxBrand &t_Brand, const ExtAxType &t_Type)
{
    std::vector<uint> result;
    auto iter = m_Database_ExtAx.find(t_Brand);
    if(iter != m_Database_ExtAx.end()){
        for(auto item : iter->second){
            if(item.model_Type == t_Type){
                if(std::find(result.begin(), result.end(), item.model_dof) == result.end()){
                    result.push_back(item.model_dof);
                }
            }
        }
    }
    return result;
}

const std::vector<float> MechanicDatabase::getAvailableRobot_Payload(const RobotBrand &t_Brand, const RobotType &t_Type, const uint t_Dof)
{
    std::vector<float> result;
    auto iter = m_Database_Robot.find(t_Brand);
    if(iter != m_Database_Robot.end()){
        for(auto item : iter->second){
            if(item.model_Type == t_Type &&
               item.model_dof == t_Dof){
                if(std::find(result.begin(), result.end(), item.model_payload) == result.end()){
                    result.push_back(item.model_payload);
                }
            }
        }
    }
    return result;
}

const std::vector<float> MechanicDatabase::getAvailablePoser_Payload(const PoserBrand &t_Brand, const PoserType &t_Type, const uint t_Dof)
{
    std::vector<float> result;
    auto iter = m_Database_Poser.find(t_Brand);
    if(iter != m_Database_Poser.end()){
        for(auto item : iter->second){
            if(item.model_Type == t_Type &&
               item.model_dof == t_Dof){
                if(std::find(result.begin(), result.end(), item.model_payload) == result.end()){
                    result.push_back(item.model_payload);
                }
            }
        }
    }
    return result;
}

const std::vector<float> MechanicDatabase::getAvailableExtAx_Payload(const ExtAxBrand &t_Brand, const ExtAxType &t_Type, const uint t_Dof)
{
    std::vector<float> result;
    auto iter = m_Database_ExtAx.find(t_Brand);
    if(iter != m_Database_ExtAx.end()){
        for(auto item : iter->second){
            if(item.model_Type == t_Type &&
               item.model_dof == t_Dof){
                if(std::find(result.begin(), result.end(), item.model_payload) == result.end()){
                    result.push_back(item.model_payload);
                }
            }
        }
    }
    return result;
}

const std::vector<RobotModelInfo> MechanicDatabase::getParaMatched_Robots(const RobotBrand &t_Brand, const RobotType &t_Type, const uint t_dof, const float t_payload) const
{
    std::vector<RobotModelInfo> result;
    auto iter = m_Database_Robot.find(t_Brand);
    if(iter != m_Database_Robot.end()){
        for(auto item : iter->second){
            if(item.model_Type == t_Type &&
               t_dof==0?true:item.model_dof == t_dof &&
               t_payload==0?true:item.model_payload == t_payload){
                result.push_back(item);
            }
        }
    }
    return result;
}

const std::vector<PoserModelInfo> MechanicDatabase::getParaMatched_Posers(const PoserBrand &t_Brand, const PoserType &t_Type, const uint t_dof, const float t_payload) const
{
    std::vector<PoserModelInfo> result;
    auto iter = m_Database_Poser.find(t_Brand);
    if(iter != m_Database_Poser.end()){
        for(auto item : iter->second){
            if(item.model_Type == t_Type &&
               t_dof==0?true:item.model_dof == t_dof &&
               t_payload==0?true:item.model_payload == t_payload){
                result.push_back(item);
            }
        }
    }
    return result;
}

const std::vector<ExtAxModelInfo> MechanicDatabase::getParaMatched_ExtAxs(const ExtAxBrand &t_Brand, const ExtAxType &t_Type, const uint t_dof, const float t_payload) const
{
    std::vector<ExtAxModelInfo> result;
    auto iter = m_Database_ExtAx.find(t_Brand);
    if(iter != m_Database_ExtAx.end()){
        for(auto item : iter->second){
            if(item.model_Type == t_Type &&
               t_dof==0?true:item.model_dof == t_dof &&
               t_payload==0?true:item.model_payload == t_payload){
                result.push_back(item);
            }
        }
    }
    return result;
}


void MechanicDatabase::append_RobotData()
{
    auto resPath = App::GetApplication().getResourceDir();

    std::vector<RobotModelInfo> kawasaki_models;
    kawasaki_models.push_back(RobotModelInfo(std::string("ba006n"), RobotType::_Standard, 6, 5.0, resPath+std::string("Mod/Robot/Lib/RobotModel/Kawasaki/BA006N/kawasaki_ba006n.urdf")));
    kawasaki_models.push_back(RobotModelInfo(std::string("ba006n_l"), RobotType::_Standard, 6, 7.0, resPath+std::string("")));
    kawasaki_models.push_back(RobotModelInfo(std::string("rs007l"), RobotType::_Standard, 6, 5.0, resPath+std::string("Mod/Robot/Lib/RobotModel/Kawasaki/RS007L/RS007L.urdf")));

    std::vector<RobotModelInfo> moka_models;
    moka_models.push_back(RobotModelInfo(std::string("MR07S_930"), RobotType::_Standard, 6, 7.0, resPath+std::string("Mod/Robot/Lib/RobotModel/MOKA/MR07S-930/MR07S_930.urdf")));
    moka_models.push_back(RobotModelInfo(std::string("MR10_1440"), RobotType::_Standard, 6, 10.0, resPath+std::string("Mod/Robot/Lib/RobotModel/MOKA/MR10-1440/MR10_1440.urdf")));
    moka_models.push_back(RobotModelInfo(std::string("MR12_2010"), RobotType::_Standard, 6, 12.0, resPath+std::string("Mod/Robot/Lib/RobotModel/MOKA/MR12-2010/MR12_2010.urdf")));

    m_Database_Robot.insert(std::make_pair(RobotBrand::Kawasaki,kawasaki_models));
    m_Database_Robot.insert(std::make_pair(RobotBrand::MOKA,moka_models));
}

void MechanicDatabase::append_PoserData()
{
    std::vector<PoserModelInfo> Meric_models;
    Meric_models.push_back(PoserModelInfo(std::string("Meric_2LPositioner"), PoserType::_LType, 2, 100.0,std::string("/home/yix/model/Poser/Meric/2Dof/_LType/_200kg/L_Type_2AxisPositioner_NoGripper.urdf")));
    Meric_models.push_back(PoserModelInfo(std::string("Meric_2HPositioner"), PoserType::_LType, 2, 900.0,std::string("/home/yix/model/Poser/Meric/2Dof/_LType/_500kg/2AxisPositioner.urdf")));

    m_Database_Poser.insert(std::make_pair(PoserBrand::Meric, Meric_models));
}

void MechanicDatabase::append_ExtAxData()
{
    std::vector<ExtAxModelInfo> Customized_models;
    Customized_models.push_back(ExtAxModelInfo(std::string("Fork_Lift"), ExtAxType::_ComposeSystem, 2, 500.0, std::string("")));

    m_Database_ExtAx.insert(std::make_pair(ExtAxBrand::Customize, Customized_models));
}

