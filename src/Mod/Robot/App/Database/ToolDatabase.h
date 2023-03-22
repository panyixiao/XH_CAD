// Created By Yixiao 2023-3-15

#ifndef _TOOLDATABASE_
#define _TOOLDATABASE_

#include <Base/Vector3D.h>
#include <vector>
#include <map>

#include <string>
#include <Base/Placement.h>

#define stringify(name) #name

namespace Robot
{

// RobotModel Item
enum class Brand_WeldTorch{
    Telma = 0
};
static const char* StrList_WeldTorchBrands[]{
    "Telma"
};
enum class Type_TorchTube{
  Bended = 0,
  Straight,
  Undefined
};
static const char* StrList_TorchTubeType[]{
    "Bended",
    "Straight"
};

struct ToolInfo_WeldTorch{
    ToolInfo_WeldTorch(std::string _name = "",
                       Type_TorchTube _type = Type_TorchTube::Undefined,
                       uint tubelength = 0.0,
                       std::string _filepath = ""):
        torch_Name(_name),
        tube_Type(_type),
        tube_Length(tubelength),
        model_FilePath(_filepath)
    {}
    Type_TorchTube tube_Type;
    std::string torch_Name;
    std::string model_FilePath;
    uint tube_Length;
};

// RobotModel Item
enum class Brand_2DScanner{
    Googol = 0,
    HikVision
};
static const char* StrList_2DScannerBrands[]{
    "Googol",
    "HikVision"
};
enum class Type_MountPose{
  Horizontal = 0,
  Vertical,
  Tilted,
  Undefined
};
static const char* StrList_2DScannerMount[]{
    "Horizontal",
    "Vertical",
    "Tilted"
};

struct ToolInfo_2DScanner{
    ToolInfo_2DScanner(std::string _name,
                       Type_MountPose _type,
                       uint scanRange,
                       std::string _filepath):
        scanner_Name(_name),
        m_MountPose(_type),
        m_scanRange(scanRange),
        scanner_ModelPath(_filepath)
    {}
    std::string scanner_Name;
    std::string scanner_ModelPath;
    uint m_scanRange;
    Type_MountPose m_MountPose;
};

class ToolDatabase
{
public:
	/// Constructor
    ToolDatabase(void);
    virtual ~ToolDatabase();
    const std::string getResFilePath();
//    const ToolInfo_WeldTorch getToolInfo_WeldTorch(const std::string t_brandName,
//                                                   const std::string t_ModelName);
//    const ToolInfo_2DScanner getToolInfo_2DScanner(const std::string t_brandName,
//                                                   const std::string t_ModelName);

    const std::vector<Brand_WeldTorch> getAvailableTorchBrands() const;
    const std::vector<Type_TorchTube> getAvailableTorchTubeType(const Brand_WeldTorch &t_Brand);
    const std::vector<float> getAvailableTorchTubeLength(const Brand_WeldTorch &t_Brand,
                                                         const Type_TorchTube &t_Type);
    const std::vector<ToolInfo_WeldTorch> getParaMatchedTool_WeldTorch(const Brand_WeldTorch &t_Brand,
                                                                       const Type_TorchTube &t_Type,
                                                                       const float t_Tubelength) const;

    const std::vector<Brand_2DScanner> getAvailableScannerBrands() const;
    const std::vector<Type_MountPose> getAvailabeScannerMountPose(const Brand_2DScanner &t_Brand);
    const std::vector<float> getAvailableScannerRange(const Brand_2DScanner &t_Brand, const Type_MountPose &t_Type);
    const std::vector<ToolInfo_2DScanner> getParaMatchedTool_2DScanner(const Brand_2DScanner &t_Brand,
                                                                       const Type_MountPose &t_Type,
                                                                       const float t_ScannerRange) const;
    static Type_MountPose findMountPose(const std::string str_mountType);


protected:
    void appendToolDatabase_weldTorch();
    void appendToolDatabase_2DScanner();

private:
    std::string m_resPath;
    std::map<Brand_WeldTorch, std::vector<ToolInfo_WeldTorch>> m_Database_Torch;
    std::map<Brand_2DScanner, std::vector<ToolInfo_2DScanner>> m_Database_Scanner;
};

}



#endif
