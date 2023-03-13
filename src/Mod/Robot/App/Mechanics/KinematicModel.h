
#ifndef ROBOT_KINEMATICMODEL_H
#define ROBOT_KINEMATICMODEL_H

#include <Mod/Robot/App/PreCompiled.h>
#include "Mod/Robot/App/kdl_cp/chain.hpp"
#include "Mod/Robot/App/kdl_cp/chainfksolverpos_recursive.hpp"
#include "Mod/Robot/App/kdl_cp/frames_io.hpp"
#include "Mod/Robot/App/kdl_cp/chainiksolvervel_pinv.hpp"
#include "Mod/Robot/App/kdl_cp/chainjnttojacsolver.hpp"
#include "Mod/Robot/App/kdl_cp/chainiksolverpos_nr.hpp"
#include "Mod/Robot/App/kdl_cp/chainiksolverpos_nr_jl.hpp"
#include "Mod/Robot/App/trac_ik_cp/trac_ik.hpp"

#include "Mod/Robot/App/kdl_cp/chain.hpp"
#include "Mod/Robot/App/kdl_cp/jntarray.hpp"
#include "Mod/Robot/App/kdl_parser_cp/kdl_parser.hpp"

#include <Base/Persistence.h>
#include <Base/Placement.h>

#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace Robot
{
 /// NON: No Config Limit
 /// Wrist: F-lip / N-oFlip
 /// Arm  : L-eft / R-ight
 /// Elbow: U-p   / D-own
enum class ConfigType{
    NON = 0,
    NRU,
    FRU,
    NLU,
    FLU,
    NRD,
    FRD,
    NLD,
    FLD,
};

enum class MechanicType{
    M_Robot = 0,
    M_Positioner,
    M_ExtAxis,
    M_Group
};

enum class JointType{
    Rotation = 0,
    Translation
};

enum TeachCoord{
    World = 0,
    RobotBase,
    Tip
};

using ExtAxis = std::pair<JointType, double>;

/// Definition of the Axis properties
struct AxisDefinition {
    double a;        // a of the Denavit-Hartenberg parameters (mm) 
    double alpha;    // alpha of the Denavit-Hartenberg parameters (°)
    double d;        // d of the Denavit-Hartenberg parameters (mm)
    double theta;    // a of the Denavit-Hartenberg parameters (°) 
    double rotDir;   // rotational direction (1|-1)
    double maxAngle; // soft ends + in °
    double minAngle; // soft ends - in ° 
    double velocity; // max vlocity of the axle in °/s
};

enum class IK_SolverType{
    TRAC_IK,
    KDL_IK,
    IKFAST_IK
};

class RobotExport KinematicModel : public Base::Persistence
{
    TYPESYSTEM_HEADER();

public:
    KinematicModel();
    ~KinematicModel();

	// from base class
    virtual unsigned int getMemSize (void) const;
	virtual void Save (Base::Writer &/*writer*/) const;
    virtual void Restore(Base::XMLReader &/*reader*/);
    const bool initialized() const;
    uint getJointNumbers() const{
        return m_KDLchain.getNrOfJoints();
    }
    void setMechanicType(const MechanicType& t_Type);
    /// set the kinematic parameters of the robot
    bool setupJointChain(const std::vector<Base::Placement>& axisPoses);
    /// read the kinematic parameters of the robot from a file
    bool readURDFfiles(const char* FileName);
    bool updateKDLChain();

    void setIKsolverType(const IK_SolverType t_Type);
    /// set the robot to that position, calculates the Axis
    bool setTo(const Base::Placement &t_Pose);
    bool flipJointDir(uint j_ID, bool invert);
    bool JointDirInverted(uint j_ID);
    bool setJointAngle(int jntID, double Value);
    bool setJointAngles(const std::vector<double> &jntAngles);
    bool setJointMaxLimits(const std::vector<double> &MaxAngles);
    bool setJointMinLimits(const std::vector<double> &MinAngles);

    const std::vector<std::string> getJointNames() const;
    const std::vector<std::string>& getLinkNames() const;
    void setJointSpeedLimit(const uint t_ID, double degPersec);
    const double getJointSpeedLimit(const uint JntID) const;
    const std::vector<double> getJointSpeedLimits() const;
    const double getJointAngle(int Axis) const;
    const double getMaxAngle(int Axis) const;
    const double getMinAngle(int Axis) const;
    const std::vector<double> getJointAngles() const;
    const std::vector<double> getJointMaxAngles() const;
    const std::vector<double> getJointMinAngles() const;
    void updateJointFrames();
    /// calculate the new Tcp out of the Axis
    const Base::Placement getTcp(void) const;
    void setKinematicModelConfig(const ConfigType& t_Type);
    const ConfigType& getConfigType() const;

    //void setKinematik(const std::vector<std::vector<float> > &KinTable);
    const Base::Placement getJointPose(int jntID) const;

protected:
    void updateIkSolvers();
    void saveChain(Base::Writer& writer) const;
    void updateJointLimitsByConfigType();

protected:
    urdf::ModelInterfaceSharedPtr m_RobotModel;

    KDL::Tree m_KDLtree;
    KDL::Chain m_KDLchain;
    KDL::JntArray m_JointValues;
    KDL::JntArray m_JointLowerLimits;
    KDL::JntArray m_JointLowerLimits_Configured;
    KDL::JntArray m_JointUpperLimits;
    KDL::JntArray m_JointUpperLimits_Configured;
    KDL::JntArray m_JointSpeedLimits;
    KDL::JntArray m_JointRotDir;

    KDL::Frame m_TCPframe;
    std::vector<KDL::Frame> jointFrameVec;

    std::vector<std::string> m_linkNames;

    KDL::ChainFkSolverPos_recursive* m_FkSolverPtr = nullptr;
    KDL::ChainIkSolverVel_pinv* m_IkVelSolverPtr = nullptr;
    KDL::ChainIkSolverPos_NR_JL* m_kdl_solver = nullptr;
    TRAC_IK::TRAC_IK* m_tracik_solver = nullptr;

    uint jointNumber;
    float trans_ratio = 1000.0;

    IK_SolverType m_solverType;
    MechanicType m_MechanicType;
    ConfigType m_ConfigType = ConfigType::NON;
};

} //namespace Part


#endif // PART_TOPOSHAPE_H
