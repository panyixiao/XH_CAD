// Created By Yixiao 2023-03-15

#ifndef _PreComp_
#endif
#include "ToolDatabase.h"
#include <stdio.h>
#include <iostream>
#include <algorithm>

#include <Base/Console.h>
#include <Base/VectorPy.h>

#include <App/Application.h>


using namespace Robot;
using namespace std;

ToolDatabase::ToolDatabase(void)
{
    m_resPath = App::GetApplication().getResourceDir() + std::string("Mod/Robot/Lib/Model/Tools/");
    appendToolDatabase_weldTorch();
    appendToolDatabase_2DScanner();
}

ToolDatabase::~ToolDatabase()
{

}

const string ToolDatabase::getResFilePath()
{
    return m_resPath;
}

//const ToolInfo_WeldTorch ToolDatabase::getToolInfo_WeldTorch(const std::string t_brandName, const std::string t_ModelName)
//{

//}

void ToolDatabase::appendToolDatabase_weldTorch()
{
    std::vector<ToolInfo_WeldTorch> weldTorch_Telma;
    weldTorch_Telma.push_back(ToolInfo_WeldTorch(std::string("Telma_220_Bended"),Type_TorchTube::Bended,220.0,m_resPath+std::string("Torch/Torch_220.stp")));
    weldTorch_Telma.push_back(ToolInfo_WeldTorch(std::string("Telma_350_Bended"),Type_TorchTube::Bended,350.0,m_resPath+std::string("Torch/Torch_350.stp")));
    weldTorch_Telma.push_back(ToolInfo_WeldTorch(std::string("Telma_430_Bended"),Type_TorchTube::Bended,430.0,m_resPath+std::string("Torch/Torch_430.stp")));
    weldTorch_Telma.push_back(ToolInfo_WeldTorch(std::string("Telma_440_Bended"),Type_TorchTube::Bended,440.0,m_resPath+std::string("Torch/Torch_440.stp")));
    weldTorch_Telma.push_back(ToolInfo_WeldTorch(std::string("Telma_180_Straight"),Type_TorchTube::Straight,180.0,m_resPath+std::string("Torch/Torch_180.stp")));
    m_Database_Torch.insert(std::make_pair(Brand_WeldTorch::Telma,weldTorch_Telma));
}

void ToolDatabase::appendToolDatabase_2DScanner()
{
    std::vector<ToolInfo_2DScanner> _2dScanner_Googol;
    _2dScanner_Googol.push_back(ToolInfo_2DScanner(std::string("LaserScanner_90Degree"),Type_MountPose::Vertical, 400.0, m_resPath + std::string("Sensor/LaserScanner/ScannerV1_withInstall.step")));
    _2dScanner_Googol.push_back(ToolInfo_2DScanner(std::string("LaserScanner_45Degree"),Type_MountPose::Tilted, 400.0, m_resPath + std::string("Sensor/LaserScanner/ScannerV2_withInstall.step")));
    m_Database_Scanner.insert(std::make_pair(Brand_2DScanner::Googol, _2dScanner_Googol));
}

const std::vector<Brand_WeldTorch> ToolDatabase::getAvailableTorchBrands() const
{
    std::vector<Brand_WeldTorch> brands;
    for(auto iter = m_Database_Torch.begin(); iter!=m_Database_Torch.end();iter++){
        if(std::find_if(brands.begin(), brands.end(), [&](const Brand_WeldTorch& t_brand){
                        return iter->first == t_brand;}) == brands.end()){
            brands.push_back(iter->first);
        }
    }
    return brands;
}

const std::vector<Brand_2DScanner> ToolDatabase::getAvailableScannerBrands() const
{
    std::vector<Brand_2DScanner> brands;
    for(auto iter = m_Database_Scanner.begin(); iter!=m_Database_Scanner.end();iter++){
        if(std::find_if(brands.begin(), brands.end(), [&](const Brand_2DScanner& t_brand){
                        return iter->first == t_brand;}) == brands.end()){
            brands.push_back(iter->first);
        }
    }
    return brands;
}

const std::vector<Type_TorchTube> ToolDatabase::getAvailableTorchTubeType(const Brand_WeldTorch &t_Brand)
{
    std::vector<Type_TorchTube> result;
    auto iter = m_Database_Torch.find(t_Brand);
    if(iter != m_Database_Torch.end()){
        for(auto item : iter->second){
            if(std::find_if(result.begin(), result.end(), [&](const Type_TorchTube& t_Type){
                            return t_Type == item.tube_Type;}) == result.end()){
                result.push_back(item.tube_Type);
            }
        }
    }
    return result;
}

const std::vector<Type_MountPose> ToolDatabase::getAvailabeScannerMountPose(const Brand_2DScanner &t_Brand)
{
    std::vector<Type_MountPose> result;
    auto iter = m_Database_Scanner.find(t_Brand);
    if(iter != m_Database_Scanner.end()){
        for(auto item : iter->second){
            if(std::find_if(result.begin(), result.end(), [&](const Type_MountPose& t_Type){
                            return t_Type == item.m_MountPose;}) == result.end()){
                result.push_back(item.m_MountPose);
            }
        }
    }
    return result;
}


const std::vector<float> ToolDatabase::getAvailableTorchTubeLength(const Brand_WeldTorch &t_Brand, const Type_TorchTube &t_Type)
{
    std::vector<float> result;
    auto iter = m_Database_Torch.find(t_Brand);
    if(iter != m_Database_Torch.end()){
        for(auto item : iter->second){
            if(item.tube_Type == t_Type){
                if(std::find(result.begin(), result.end(), item.tube_Length) == result.end()){
                    result.push_back(item.tube_Length);
                }
            }
        }
    }
    return result;
}

const std::vector<float> ToolDatabase::getAvailableScannerRange(const Brand_2DScanner &t_Brand, const Type_MountPose &t_Type)
{
    std::vector<float> result;
    auto iter = m_Database_Scanner.find(t_Brand);
    if(iter != m_Database_Scanner.end()){
        for(auto item : iter->second){
            if(item.m_MountPose == t_Type){
                if(std::find(result.begin(), result.end(), item.m_scanRange) == result.end()){
                    result.push_back(item.m_scanRange);
                }
            }
        }
    }
    return result;
}

const std::vector<ToolInfo_WeldTorch> ToolDatabase::getParaMatchedTool_WeldTorch(const Brand_WeldTorch &t_Brand, const Type_TorchTube &t_Type, const float t_Tubelength) const
{
    std::vector<ToolInfo_WeldTorch> result;
    auto iter = m_Database_Torch.find(t_Brand);
    if(iter != m_Database_Torch.end()){
        for(auto item : iter->second){
            if(item.tube_Type == t_Type &&
               t_Tubelength==0?true:item.tube_Length == t_Tubelength){
                result.push_back(item);
            }
        }
    }
    return result;
}

const std::vector<ToolInfo_2DScanner> ToolDatabase::getParaMatchedTool_2DScanner(const Brand_2DScanner &t_Brand, const Type_MountPose &t_Type, const float t_ScannerRange) const
{
    std::vector<ToolInfo_2DScanner> result;
    auto iter = m_Database_Scanner.find(t_Brand);
    if(iter != m_Database_Scanner.end()){
        for(auto& item : iter->second){
            if(item.m_MountPose == t_Type &&
               t_ScannerRange ==0?true:item.m_scanRange == t_ScannerRange){
                result.push_back(item);
            }
        }
    }
    return result;
}

Type_MountPose ToolDatabase::findMountPose(const std::string str_mountType)
{
    for(size_t i = 0; i<std::size(StrList_2DScannerMount); i++){
        if(std::strcmp(str_mountType.c_str(), StrList_2DScannerMount[i]) == 0)
            return (Type_MountPose)i;
    }
    return Type_MountPose::Undefined;
}
