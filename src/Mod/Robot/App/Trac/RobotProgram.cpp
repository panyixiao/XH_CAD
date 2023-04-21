/***************************************************************************
 *   Copyright (c) Jürgen Riegel          (juergen.riegel@web.de) 2002     *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/


#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Exception.h>
#include "RobotProgram.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"

#ifndef M_PI
    #define M_PI    3.14159265358979323846
    #define M_PI    3.14159265358979323846 /* pi */
#endif

#ifndef M_PI_2
    #define M_PI_2  1.57079632679489661923 /* pi/2 */
#endif

using namespace Robot;
using namespace Base;
//using namespace KDL;

TYPESYSTEM_SOURCE(Robot::RobotProgram , Base::Persistence);

RobotProgram::RobotProgram()
    :m_ProgramID(boost::uuids::random_generator()())
{

}

RobotProgram::RobotProgram(const RobotProgram& Trac)
: m_waypoints(Trac.m_waypoints.size()),
  m_ProgramID(boost::uuids::random_generator()())
{
    operator=(Trac);
}

RobotProgram::~RobotProgram()
{
    clearData();
}

RobotProgram &RobotProgram::operator=(const RobotProgram& Trac)
{
    this->m_waypoints.clear();
    this->m_waypoints = Trac.m_waypoints;

    this->m_commands = Trac.m_commands;

//    this->m_TracOriginPose = Trac.m_TracOriginPose;
//    this->m_ExecutorOriginPose = Trac.m_ExecutorOriginPose;
//    updateSimTracData();

//    this->_taskName = Trac._taskName;
//    this->_operatorName = Trac._operatorName;
//    this->_type = Trac._type;

    return *this;

}


//void RobotProgram::deleteLast(unsigned int n)
//{
//    for(unsigned int i=0;i<=n;i++){
//        m_poseData.pop_back();
//    }
//}



void RobotProgram::setExecutorBase(const Placement &t_Pose)
{
//    m_ExecutorOriginPose = t_Pose;
}

void RobotProgram::setTracDataOrigin(const Placement &t_Pose)
{
//    m_TracOriginPose = t_Pose;
    // 更新所有点位置
}

void RobotProgram::insertCMD_NewMOVE(const std::string executorName,
                                     const std::size_t t_PoseID,
                                     const MoveType t_Type,
                                     const MovePrec t_Prec,
                                     const float t_V,
                                     const float t_BL,
                                     const float t_VBL)
{
    m_commands.push_back(std::make_shared<Robot::MoveCommand>(Robot::MoveCommand(_taskName,
                                                                                executorName,
                                                                                t_Type,
                                                                                t_Prec,
                                                                                t_PoseID,
                                                                                t_V,t_BL,t_VBL)));
}

void RobotProgram::insertCMD_SwitchTool(const string executorName,
                                        const ToolType &t_Type,
                                        const uint coordID)
{
    m_commands.push_back(std::make_shared<Robot::ToolCommand>(Robot::ToolCommand(_taskName,
                                                                                executorName,
                                                                                t_Type,
                                                                                coordID)));
}

void RobotProgram::insertCMD_SetToolOn(const std::string executorName,
                                       const Robot::ToolType& t_Type)
{
    m_commands.push_back(std::make_shared<Robot::ToolCommand>(Robot::ToolCommand(_taskName,
                                                                                executorName,
                                                                                t_Type,
                                                                                true)));
}

void RobotProgram::insertCMD_SetToolOff(const string executorName, const ToolType &t_Type)
{
    m_commands.push_back(std::make_shared<Robot::ToolCommand>(Robot::ToolCommand(_taskName,
                                                                                executorName,
                                                                                t_Type,
                                                                                false)));
}

void RobotProgram::insertCMD_ChgCord(const std::string executorName, const CordType &t_Type, const uint cordID)
{
    m_commands.push_back(std::make_shared<Robot::CoordCommand>(Robot::CoordCommand(_taskName,
                                                                                  executorName,
                                                                                  t_Type,
                                                                                  cordID)));
}

const std::size_t RobotProgram::addNewPose(const RobotWaypoint &t_Pnt)
{
    auto pntID = findWaypoint(t_Pnt.getPointHashID());
    if(pntID>0)
        return pntID;
    auto tmp = std::make_shared<RobotWaypoint>(t_Pnt);
    m_waypoints.push_back(tmp);
    return tmp->getPointHashID();
}

#include <algorithm>
const std::size_t RobotProgram::findWaypoint(const size_t pointID)
{
    std::size_t result = 0;
    auto iter = std::find_if(m_waypoints.cbegin(),m_waypoints.cend(),[&](const RobotWaypoint_sptr& c_iter){
        return c_iter->getPointHashID() == pointID;
    });
    if(iter != m_waypoints.cend()){
        return pointID;
    }
    return result;
}

bool RobotProgram::deleteCMD_byPosition(const uint position)
{
    if(position > m_commands.size())
        return false;
    auto t_cmdPtr = getCMD_byPosition(position);
    if(t_cmdPtr->getType() == Robot::CommandType::SetMove){
        auto t_MovPtr = static_cast<Robot::MoveCommand*>(t_cmdPtr.get());
        int count = 0;
        for(auto& cmdPtr : m_commands){
            if(cmdPtr->getType() == Robot::CommandType::SetMove){
                auto movCmdPtr = static_cast<Robot::MoveCommand*>(cmdPtr.get());;
                if(movCmdPtr->getMovPoseID() == t_MovPtr->getMovPoseID())
                    count++;
            }
        }
        // The last command that bind to this pose
        if(count == 1){
            // Delete Point from poseData
            deleteWP_byHashID(t_MovPtr->getMovPoseID());
        }
    }
    auto iter = m_commands.begin() + position;
    m_commands.erase(iter);

    return true;
}

const RobotCommand_sptr RobotProgram::getCMD_byPosition(const uint position)
{
    if(position>m_commands.size())
        return nullptr;
    return m_commands.at(position);
}

const std::vector<RobotCommand_sptr> RobotProgram::getCmmdData(const uint s_ID) const
{
    std::vector<RobotCommand_sptr> result;
    if(s_ID>m_commands.size())
        return result;
    result.assign(m_commands.begin()+s_ID, m_commands.end());
    return result;
}

const std::string RobotProgram::generateCommandStr(RobotCommand_sptr t_CommandPtr) const
{
//    RobotCommand tmp_Cmd = t_Command;
    std::string cmd_str;
    if(t_CommandPtr == nullptr)
        return cmd_str;
    switch(t_CommandPtr->getType()){
    case CommandType::SetMove:
        cmd_str = generateCMDstr_Move(t_CommandPtr);
        break;
    case CommandType::ChgTool:
    case CommandType::OptTool:
        cmd_str = generateCMDstr_Tool(t_CommandPtr);
        break;
    case CommandType::SetCord:
        cmd_str = generateCMDstr_Cord(t_CommandPtr);
        break;
    default:
        break;
    }

    return cmd_str;
}

const std::string RobotProgram::generateCMDstr_Move(RobotCommand_sptr t_Command) const
{
    std::string result_str;
    auto cmd_ptr = static_cast<Robot::MoveCommand*>(t_Command.get());
    auto pntPosition = getPosePosition(cmd_ptr->getMovPoseID());

    switch(cmd_ptr->getMoveType()){
    case MoveType::MOVL:
        result_str += "MOVL ";
        break;
    case MoveType::MOVJ:
        result_str += "MOVJ ";
        break;
    case MoveType::MOVC:
        result_str += "MOVC ";
        break;
    default:
        result_str += "MOVX ";
        break;
    }

    result_str += "P["+std::to_string(pntPosition) +"] ";

    switch (cmd_ptr->getMovePrec()) {
    case MovePrec::FINE:
        result_str += "FINE ";
        break;
    case MovePrec::CNT:
        result_str += "CNT ";
    default:
        break;
    }
    auto param = cmd_ptr->getMoveParam();
    if(param.size() == 3){
        result_str += "V=" + DS_Utility::double2string(param[0],1);
        if(cmd_ptr->getMoveType() == MoveType::MOVJ)
            result_str += "% ";
        else
            result_str += "mm/sec ";

        result_str += "BL=" + DS_Utility::double2string(param[1],1);
        result_str += " VBL=" + DS_Utility::double2string(param[2],1);
    }

    return result_str;
}

const std::string RobotProgram::generateCMDstr_Cord(RobotCommand_sptr t_Command) const
{
    std::string result_str;
    auto cmd_ptr = static_cast<Robot::CoordCommand*>(t_Command.get());
    result_str += "Set Coord ";
    switch(cmd_ptr->getCoordType()){
    case CordType::PCS:
        result_str += "Type=PCS ";
        result_str += "ID="+ std::to_string(cmd_ptr->getCoordID());
        break;
    case CordType::MCS:
        result_str += "Type=MCS";
        break;
    case CordType::ACS:
        result_str += "Type=ACS";
        break;
    case CordType::TCS:
        result_str += "Type=TCS";
        result_str += "ID="+ std::to_string(cmd_ptr->getCoordID());
        break;
    case CordType::VCS:
        result_str += "Coord_Num=4";
        break;
    }

    return result_str;
}

const std::string RobotProgram::generateCMDstr_Tool(RobotCommand_sptr t_Command) const
{
    std::string result_str;
    bool hasStatus = false;
    auto cmd_ptr = static_cast<Robot::ToolCommand*>(t_Command.get());
    if(cmd_ptr->getType() == Robot::CommandType::ChgTool){
        switch(cmd_ptr->getToolType()){
        case ToolType::Undefined:
            result_str+="SetTool=Flan";
            break;
        case ToolType::WeldTorch:
            result_str+="SetTool=Torch";
            hasStatus = true;
            break;
        case ToolType::_2DScanner:
            result_str+="SetTool=Scanner";
            hasStatus = true;
            break;
        }
    }
    else if(cmd_ptr->getType() == Robot::CommandType::OptTool){
        switch(cmd_ptr->getToolType()){
        case ToolType::Undefined:
            break;
        case ToolType::WeldTorch:
            result_str+="Arc";
            hasStatus = true;
            break;
        case ToolType::_2DScanner:
            result_str+="Laser";
            hasStatus = true;
            break;
        }
        if(hasStatus){
            if(cmd_ptr->getToolStatus())
                result_str+=" On";
            else
                result_str+=" Off";
        }
    }
    return result_str;
}

void RobotProgram::joinData(const std::shared_ptr<RobotProgram> t_ProgramPtr)
{
    if(t_ProgramPtr == nullptr)
        return;
    joinCmmdData(t_ProgramPtr->getCmmdData());
    joinPoseData(t_ProgramPtr->getWaypointData());
}

unsigned int RobotProgram::getMemSize (void) const
{
    return m_waypoints.size();
}

void RobotProgram::Save (Writer &writer) const
{
    writer.Stream() << writer.ind() << "<Poses Count=\"" <<  m_waypoints.size() <<"\">" << std::endl;
    writer.incInd();
    for(unsigned int i = 0;i<m_waypoints.size(); i++)
        m_waypoints[i]->Save(writer);
    writer.decInd();
    writer.Stream() << writer.ind() << "</Poses>" << std::endl ;

    writer.Stream() << writer.ind() << "<Commands Count=\"" <<  m_commands.size() <<"\">" << std::endl;
    writer.incInd();
    for(unsigned int i = 0;i<m_commands.size(); i++)
        m_commands[i]->Save(writer);
    writer.decInd();
    writer.Stream() << writer.ind() << "</Commands>" << std::endl ;

}

void RobotProgram::Restore(XMLReader &reader)
{
    m_waypoints.clear();
    // read my element
    reader.readElement("Poses");
    // get the value of my Attribute
    int poseNum = reader.getAttributeAsInteger("Count");
    m_waypoints.resize(poseNum);
    for (int i = 0; i < poseNum; i++) {
        auto t_waypoint = std::make_shared<RobotWaypoint>();
        t_waypoint->Restore(reader);
        m_waypoints[i] = t_waypoint;
    }
    reader.readEndElement("Poses");

    m_commands.clear();
    // read my element
    reader.readElement("Commands");
    int cmdNum = reader.getAttributeAsInteger("Count");
//    m_cmdData.resize(cmdNum);
    for (int i = 0; i < cmdNum; i++) {
        reader.readElement("Command");
        auto cmdType = (CommandType)reader.getAttributeAsInteger("CmdType");
        switch(cmdType){
        case CommandType::SetCord:{
                auto t_command = std::make_shared<CoordCommand>();
                t_command->setCommandType(cmdType);
                t_command->Restore(reader);
                m_commands.push_back(t_command);
                break;
            }
        case CommandType::SetMove:
            {
                auto t_command = std::make_shared<MoveCommand>();
                t_command->setCommandType(cmdType);
                t_command->Restore(reader);
                m_commands.push_back(t_command);
                break;
            }
        case CommandType::ChgTool:
        case CommandType::OptTool:
            {
                auto t_command = std::make_shared<ToolCommand>();
                t_command->setCommandType(cmdType);
                t_command->Restore(reader);
                m_commands.push_back(t_command);
                break;
            }
        }
    }
    reader.readEndElement("Commands");
}

bool RobotProgram::isProgramValid()
{
    if(m_commands.empty())
        return false;
    bool validFlag = true;
    for(const auto& t_cmdPtr : m_commands){
        if(t_cmdPtr->getType() == CommandType::SetMove){
            validFlag &= getWaypoint_byCommand(t_cmdPtr)->isValid();
        }
    }
    return validFlag;
}

void RobotProgram::joinPoseData(const std::vector<RobotWaypoint_sptr> &t_PoseData)
{
    auto originSize = m_waypoints.size();
    m_waypoints.resize(originSize + t_PoseData.size());
    std::copy(t_PoseData.begin(), t_PoseData.end(),m_waypoints.begin()+originSize);
}

void RobotProgram::joinCmmdData(const std::vector<RobotCommand_sptr> &t_CmmdData)
{
    auto originSize = m_commands.size();
    m_commands.resize(originSize + t_CmmdData.size());
    std::copy(t_CmmdData.begin(), t_CmmdData.end(),m_commands.begin()+originSize);
}

void RobotProgram::clearData()
{
    m_commands.clear();
    m_waypoints.clear();
}

void RobotProgram::setWPntCartPose(const int pntID,
                                      const uint rbtID,
                                      const Placement &new_Pose)
{
    auto& t_PntIter = m_waypoints.at(pntID);
    if(rbtID == 1)
        t_PntIter->setGp1_CartPose(new_Pose);
    if(rbtID == 2)
        t_PntIter->setGp2_CartPose(new_Pose);
}

void RobotProgram::setWPntPoseAdjust(const int pntID,
                                        const uint rbtID,
                                        const Placement &new_Adjust)
{
    auto& t_PntIter = m_waypoints.at(pntID);
    if(rbtID == 1)
        t_PntIter->setGp1_CartPoseAdjust(new_Adjust);
    else if(rbtID == 2)
        t_PntIter->setGp2_CartPoseAdjust(new_Adjust);
}

const RobotWaypoint_sptr RobotProgram::getWaypoint_byID(const size_t wp_id) const
{
    auto poseNum = getPosePosition(wp_id);
    return getWaypoint_byPosition(poseNum);
}

const std::size_t RobotProgram::getPosePosition(const size_t poseID) const
{
    auto iter = std::find_if(m_waypoints.cbegin(),m_waypoints.cend(),[&](const RobotWaypoint_sptr& t_Pose){
        return t_Pose->getPointHashID() == poseID;
    });

    if(iter != m_waypoints.cend())
        return std::distance(m_waypoints.cbegin(),iter);
    return -1;
}

const RobotWaypoint_sptr RobotProgram::getWaypoint_byCommand(RobotCommand_sptr t_CommandPtr) const
{
    if(t_CommandPtr->getType() != CommandType::SetMove)
        return nullptr;
    auto movCmdPtr = static_cast<Robot::MoveCommand*>(t_CommandPtr.get());
    auto position = getPosePosition(movCmdPtr->getMovPoseID());
    return getWaypoint_byPosition(position);
}

const std::vector<RobotCommand_sptr> RobotProgram::snipProgramSegment(const uint s_ID, const uint e_ID)
{
    std::vector<RobotCommand_sptr> result;
//    for(uint i = s_ID; i < e_ID; i++){
//        result.push_back(m_cmdData[i]);
//    }
    result.assign(m_commands.begin()+s_ID, m_commands.begin()+e_ID);
//    std::copy(m_cmdData.begin()+s_CMDpos, m_cmdData.begin()+e_CMDpos,result);
    return result;
}

void RobotProgram::deleteWP_byHashID(const std::size_t &t_HashID)
{
    auto iter = std::find_if(m_waypoints.begin(), m_waypoints.end(),[&](auto t_posePtr){
        return t_HashID == t_posePtr->getPointHashID();
    });
    if(iter!=m_waypoints.end())
        m_waypoints.erase(iter);
}

void RobotProgram::replacePoseData(const std::vector<RobotWaypoint_sptr> &t_PoseData)
{
    m_waypoints.clear();
    m_waypoints = t_PoseData;
}

bool RobotProgram::exportProgram(const string file_Path)
{

}




 
