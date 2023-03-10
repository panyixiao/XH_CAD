// Created By Yixiao 2022-10-13

#ifndef _MECHANICDATABASE_
#define _MECHANICDATABASE_

#include <Base/Vector3D.h>
#include <string>
#include <vector>
#include <map>

//#include <Mod/Robot/App/Utilites/EnumUtility.h>
#include <Base/Placement.h>

#define stringify(name) #name

namespace Robot
{

// ExtAxModel Item
enum class ExtAxBrand{
    Customize = 0
};
static const char* ExtAxBrand_str[]{
    "Customize"
};

enum class ExtAxType{
  _RotateStation = 0,
  _RailSystem,
  _ComposeSystem
};
static const char* ExtAxType_str[]{
    "RotateStation",
    "RailSystem",
    "ComposeSystem"
};

struct ExtAxModelInfo{
    ExtAxModelInfo(std::string _name, ExtAxType _type, uint _dof, float _payload, std::string _filepath):
        model_Name(_name), model_Type(_type), model_dof(_dof), model_payload(_payload), model_FilePath(_filepath)
    {}
    std::string model_Name;
    ExtAxType model_Type;
    uint model_dof;
    float model_payload;
    std::string model_FilePath;
};

// PoserModel Item
enum class PoserBrand{
    Meric = 0,
    Customize
};
static const char* PoserBrand_str[]{
    "Meric",
    "Customize",
    "ComposeSystem"
};

enum class PoserType{
  _LType = 0,
  _HType,
  _CType
};
static const char* PoserType_str[]{
    "LType",
    "HType",
    "CType"
};

struct PoserModelInfo{
    PoserModelInfo(std::string _name, PoserType _type, uint _dof, float _payload, std::string _filepath):
        model_Name(_name), model_Type(_type), model_dof(_dof), model_payload(_payload), model_FilePath(_filepath)
    {}
    std::string model_Name;
    PoserType model_Type;
    uint model_dof;
    float model_payload;
    std::string model_FilePath;
};

// RobotModel Item
enum class RobotBrand{
    Kawasaki = 0,
    MOKA,
    Customize
};
static const char* RobotBrand_str[]{
    "Kawasaki",
    "MOKA",
    "Customize"
};
enum class RobotType{
  _Standard = 0,
  _SCARA,
  _Collaborate,
  _Redundante
};
static const char* RobotType_str[]{
    "Standard",
    "SCARA",
    "Collaborate",
    "Redundante"
};

struct RobotModelInfo{
    RobotModelInfo(std::string _name, RobotType _type, uint _dof, float _payload, std::string _filepath):
        model_Name(_name), model_Type(_type), model_dof(_dof), model_payload(_payload), model_FilePath(_filepath)
    {}
    std::string model_Name;
    RobotType model_Type;
    uint model_dof;
    float model_payload;
    std::string model_FilePath;
};

class MechanicDatabase
{
public:
	/// Constructor
    MechanicDatabase(void);
    virtual ~MechanicDatabase();
    const RobotModelInfo getTargetRobotItemInfo(const std::string t_brandName,
                                                const std::string t_ModelName);
    const PoserModelInfo getTargetPoserItemInfo(const std::string t_brandName,
                                                const std::string t_ModelName);
    const ExtAxModelInfo getTargetExtAxItemInfo(const std::string t_brandName,
                                                const std::string t_ModelName);
    const std::vector<RobotBrand> getAvailableRobotBrands() const;
    const std::vector<PoserBrand> getAvailablePoserBrands() const;
    const std::vector<ExtAxBrand> getAvailableExtAxBrands() const;

    const std::vector<RobotType> getAvailableRobotType(const RobotBrand& t_Brand);
    const std::vector<PoserType> getAvailablePoserType(const PoserBrand& t_Brand);
    const std::vector<ExtAxType> getAvailableExtAxType(const ExtAxBrand& t_Brand);

    const std::vector<uint> getAvailableRobot_DOF(const RobotBrand& t_Brand, const RobotType& t_Type);
    const std::vector<uint> getAvailablePoser_DOF(const PoserBrand& t_Brand, const PoserType& t_Type);
    const std::vector<uint> getAvailableExtAx_DOF(const ExtAxBrand& t_Brand, const ExtAxType& t_Type);

    const std::vector<float> getAvailableRobot_Payload(const RobotBrand& t_Brand, const RobotType& t_Type,const uint t_Dof);
    const std::vector<float> getAvailablePoser_Payload(const PoserBrand& t_Brand, const PoserType& t_Type,const uint t_Dof);
    const std::vector<float> getAvailableExtAx_Payload(const ExtAxBrand& t_Brand, const ExtAxType& t_Type,const uint t_Dof);

    const std::vector<RobotModelInfo> getParaMatched_Robots(const RobotBrand& t_Brand,
                                                            const RobotType& t_Type,
                                                            const uint t_dof,
                                                            const float t_payload) const;
    const std::vector<PoserModelInfo> getParaMatched_Posers(const PoserBrand& t_Brand,
                                                            const PoserType& t_Type,
                                                            const uint t_dof,
                                                            const float t_payload) const;
    const std::vector<ExtAxModelInfo> getParaMatched_ExtAxs(const ExtAxBrand& t_Brand,
                                                            const ExtAxType& t_Type,
                                                            const uint t_dof,
                                                            const float t_payload) const;
protected:
    void append_RobotData();
    void append_PoserData();
    void append_ExtAxData();

private:
    std::map<RobotBrand, std::vector<RobotModelInfo>> m_Database_Robot;
    std::map<PoserBrand, std::vector<PoserModelInfo>> m_Database_Poser;
    std::map<ExtAxBrand, std::vector<ExtAxModelInfo>> m_Database_ExtAx;
};

} //namespace Robot



#endif
