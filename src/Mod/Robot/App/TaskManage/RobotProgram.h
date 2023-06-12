// Created By Yixiao 2022-05-10

#ifndef ROBOT_TRAC_H
#define ROBOT_TRAC_H

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <memory>

#include "Mod/Robot/App/TaskManage/TargetPoint.h"
#include "Mod/Robot/App/TaskManage/Command/CommandBase.h"
#include "Mod/Robot/App/TaskManage/Command/MoveCommand.h"
#include "Mod/Robot/App/TaskManage/Command/CoordCommand.h"
#include "Mod/Robot/App/TaskManage/Command/ToolCommand.h"

#include <App/GeoFeature.h>
#include <Base/Placement.h>
#include <Base/Persistence.h>
#include <vector>

/// Total 3 types of data:
/// 1. Pose Data: A map datastruct to store absolute poses, this is the database for Move Command
/// 2. Cmd  Data:
///    Move Command: Move action + Pose
///    Tool Command: Set Tool Status etc
///    Device Opt  Command: To change I/O status
/// 3. Trac Data: Transformed from Commands data, Can be simulated in TracSimulator

using TargetPoint_sptr = std::shared_ptr<Robot::TargetPoint>;
using RobotCommand_sptr = std::shared_ptr<Robot::CommandBase>;


namespace Robot
{
/** The DataStructure of a Trajectory
 */
class RobotExport RobotProgram : public App::GeoFeature
{
    TYPESYSTEM_HEADER();

public:
    RobotProgram();
    RobotProgram(const RobotProgram&);
    ~RobotProgram();

    RobotProgram &operator=(const RobotProgram&);

	// from base class
    virtual unsigned int getMemSize (void) const;
	virtual void Save (Base::Writer &/*writer*/) const;
    virtual void Restore(Base::XMLReader &/*reader*/);

    bool isProgramValid();
    void clearData();
    // Command Operation
    void insertCMD_NewMOVE(const std::size_t t_PoseID,
                           const Robot::MoveType t_Type,
                           const Robot::MovePrec t_Prec,
                           const float t_V,
                           const float t_BL,
                           const float t_VBL);

    void insertCMD_SwitchTool(const ToolType& t_Type, const uint coordID);
    void insertCMD_SetToolOn(const ToolType &t_Type);
    void insertCMD_SetToolOff(const ToolType &t_Type);
    void insertCMD_ChgCord(const CordType&t_Type,
                           const uint cordID);
//    void insertCMD_SetSpeed(const SpeedType& t_Type,
//                            const double t_speed);

    bool deleteCMD_byPosition(const uint position);    
    void joinData(const std::shared_ptr<RobotProgram> t_ProgramPtr);
    const RobotCommand_sptr getCMD_byPosition(const uint position);
    const std::vector<RobotCommand_sptr> getCmmdData(const uint s_ID = 0) const;
    // Command Generator
    const std::string generateCommandStr(RobotCommand_sptr t_CommandPtr) const;

    // Waypoint Data Operation
    const std::size_t addNewPose(const TargetPoint &t_Pnt);
    void deleteWP_byHashID(const std::size_t& t_HashID);

    void setWPntCartPose(const int pntID, const uint rbtID, const Base::Placement& new_Pose);
    void setWPntPoseAdjust(const int pntID, const uint rbtID,const Base::Placement& new_Adjust);

    const size_t getTotalWayPointCount(void) const{return m_waypoints.size();}
    const TargetPoint_sptr getWaypoint_byID(const size_t wp_id)const;
    const TargetPoint_sptr getWaypoint_byPosition(unsigned int pos)const {return m_waypoints[pos];}
    const TargetPoint_sptr getWaypoint_byCommand(RobotCommand_sptr t_CommandPtr) const;
    const std::vector<TargetPoint_sptr> &getWaypointData(void)const{
        return m_waypoints;
    }
    const std::vector<RobotCommand_sptr> snipProgramSegment(const uint s_ID,
                                                            const uint e_ID);

    void replacePoseData(const std::vector<TargetPoint_sptr>& t_PoseData);

    const std::size_t getProgramID() const{
        return boost::uuids::hash_value(m_ProgramID);
    }
    void setRoundupDist(const double r_d){
    }

    bool exportProgram(const std::string file_Path);

protected:
    const std::size_t getPosePosition(const size_t poseID) const;
    const std::size_t findWaypoint(const size_t pointID);
    void joinPoseData(const std::vector<TargetPoint_sptr>& t_PoseData);
    void joinCmmdData(const std::vector<RobotCommand_sptr> &t_CmmdData);
    const std::string generateCMDstr_Move(RobotCommand_sptr t_Command) const;
    const std::string generateCMDstr_Cord(RobotCommand_sptr t_Command) const;
    const std::string generateCMDstr_Tool(RobotCommand_sptr t_Command) const;

protected:
    boost::uuids::uuid m_ProgramID;
    std::string m_ExecutorName;
    // Data Buffer to store all kinds of commands, can be exported as real robot program
    std::vector<RobotCommand_sptr> m_commands;
    // Data Buffer to store waypoint data
    std::vector<TargetPoint_sptr> m_waypoints;
};

} //namespace Part


#endif // PART_TOPOSHAPE_H
