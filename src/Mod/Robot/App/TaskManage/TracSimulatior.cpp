// Created By Yixiao 2022-06-26

#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_

#endif

#include <stdio.h>
#include <iostream>

#include <Base/Console.h>


#include "TracSimulatior.h"

using namespace Robot;
using namespace std;
using namespace KDL;

//===========================================================================
// Simulation class
//===========================================================================

TracSimulator::TracSimulator()
{
}

TracSimulator::~TracSimulator()
{
}

bool TracSimulator::initDocPtr(App::Document *t_DocPtr)
{
    if(t_DocPtr == nullptr)
        return false;
    m_DocPtr = t_DocPtr;
}

bool TracSimulator::udpateGroupPoseByTime(const double t_Time)
{
    if(m_GroupPtr == nullptr){
        Base::Console().Warning("[TracSimulator]udpateGroupPoseByTime(): m_GroupPtr is nullptr!\n");
        return false;
    }
    updatePoseBuffer_ByTime(t_Time);
    return m_GroupPtr->setGroupPose(m_PoseBuffer);
}

bool TracSimulator::commandFinished(const size_t t_CmdID)
{
    if(t_CmdID >= m_MovCmdBuffer.size())
        return false;
    auto t_CmmdPtr = m_MovCmdBuffer.at(t_CmdID);
    m_GroupPtr = static_cast<Robot::MechanicGroup*>(m_DocPtr->getObject(t_CmmdPtr->getOperatorName().c_str()));
    if(m_GroupPtr == nullptr){
        Base::Console().Warning("[TracSimulator]commandFinished(): m_GroupPtr is nullptr!\n");
        return false;
    }
    auto t_MovCmmdPtr = static_cast<MoveCommand*>(t_CmmdPtr.get());
    return moveCommandFinished(t_MovCmmdPtr);
}

void TracSimulator::updateMovCommandBuffer(const std::vector<RobotCommand_sptr> &t_cmdBuffer)
{
    if(t_cmdBuffer.empty())
        return;
    m_MovCmdBuffer.clear();
    m_MovCmdBuffer = t_cmdBuffer;
    m_GroupPtr = static_cast<Robot::MechanicGroup*>(m_DocPtr->getObject(t_cmdBuffer.front()->getOperatorName().c_str()));
}


int TracSimulator::generateSimulationTrac_FromCurrentPose()
{
    if(m_TargetProgramPtr == nullptr)
        return -1;
    if(m_DocPtr==nullptr)
        return -1;
    try {
        m_SimTracBuffer = std::make_shared<GroupTrac>();

        std::shared_ptr<GroupFrame>  _1stFrame = nullptr;
        std::shared_ptr<GroupFrame>  _2ndFrame = nullptr;
        std::shared_ptr<GroupPath>   path_Buffer = nullptr;
        Robot::MechanicGroup*       t_GroupPtr = nullptr;

        float acc = DefaultAcc;

        bool firstMoveCmd = true, skipframe = false;
        bool flag_BuildingRoundPath = false;
        bool flag_PathComplete = false;
        bool flag_SwitchedTool = false;
        int pathCount = 0;
        for(auto cmd_iter : m_MovCmdBuffer)
        {
            t_GroupPtr = static_cast<Robot::MechanicGroup*>(m_DocPtr->getObject(cmd_iter->getOperatorName().c_str()));
            if(t_GroupPtr == nullptr){
                return -2;
            }
            // Generate Group Trac Based on Command
            switch(cmd_iter->getType()){
            case CommandType::SetMove:{
                auto movCmd = static_cast<Robot::MoveCommand*>(cmd_iter.get());
                auto moveParam = movCmd->getMoveParam();
                auto speed = moveParam[0];
                auto t_WPnt = m_TargetProgramPtr->getWaypoint_byCommand(cmd_iter);
                if(firstMoveCmd){
                    m_InitPose = t_GroupPtr->getCurrentGroupPose(CordType::WCS);
                    _1stFrame = std::make_shared<GroupFrame>(m_InitPose);
                    firstMoveCmd = false;
                }
                _2ndFrame = std::make_shared<GroupFrame>(t_WPnt->getWPPoseData());

                if(!skipframe){
                    if(movCmd->getMovePrec() == MovePrec::CNT){
                        if(!flag_BuildingRoundPath){
                            auto bl = moveParam[1];
                            auto vbl = moveParam[2];
                            path_Buffer = std::make_shared<GroupPath>(_1stFrame,
                                                                      _2ndFrame,
                                                                      speed,acc,
                                                                      bl,vbl);
                            flag_BuildingRoundPath = true;
                        }
                        else{
                            path_Buffer->addFrame(_2ndFrame);
                        }
                    }
                    else if(movCmd->getMovePrec() == MovePrec::FINE){
                        if(flag_BuildingRoundPath){
                            path_Buffer->addFrame(_2ndFrame);
                            path_Buffer->FinishPath(t_GroupPtr->getExtAxisSpeedLimits());
                            flag_BuildingRoundPath = false;
                        }
                        else{
                            path_Buffer = std::make_shared<GroupPath>(_1stFrame,
                                                                      _2ndFrame,
                                                                      speed,acc,
                                                                      t_GroupPtr->getExtAxisSpeedLimits());
                        }
                        flag_PathComplete = true;
                    }
                }
                if(flag_PathComplete){
                    m_SimTracBuffer->Add(path_Buffer);
                    pathCount++;
                    flag_PathComplete = false;
                }
                skipframe = false;
                _1stFrame = _2ndFrame;
                break;
            }
            case CommandType::ChgTool:{
                auto toolCmd = static_cast<Robot::ToolCommand*>(cmd_iter.get());
                t_GroupPtr->setCurrentToolType(toolCmd->getToolType());
                break;
            }
            default:
                break;
            }
        }
        return pathCount;
    }
    catch (KDL::Error &e) {
        throw Base::RuntimeError(e.Description());
    }
    return -3;
}

int TracSimulator::generateSimulationTrac_FromSelectedCommand(const int s_ID)
{
    if(m_TargetProgramPtr == nullptr)
        return -1;
    if(m_DocPtr==nullptr)
        return -1;
    try {
        m_MovCmdBuffer = m_TargetProgramPtr->getCmmdData(s_ID);
        m_SimTracBuffer = std::make_shared<GroupTrac>();

        std::shared_ptr<GroupFrame>  _1stFrame = nullptr;
        std::shared_ptr<GroupFrame>  _2ndFrame = nullptr;

        std::shared_ptr<GroupPath>   path_Buffer = nullptr;

        ToolType currentToolType;
        Robot::MechanicGroup*       t_GroupPtr = nullptr;

        float acc = DefaultAcc;

        bool firstMoveCmd = true, skipframe = false;
        bool flag_BuildingRoundPath = false;
        bool flag_PathComplete = false;
        bool flag_SwitchedTool = false;

        for(auto cmd_iter : m_MovCmdBuffer)
        {
            t_GroupPtr = static_cast<Robot::MechanicGroup*>(m_DocPtr->getObject(cmd_iter->getOperatorName().c_str()));
            if(t_GroupPtr == nullptr){
                return -2;
            }
            // Generate Group Trac Based on Command
            switch(cmd_iter->getType()){
            case CommandType::SetMove:{
                auto movCmd = static_cast<Robot::MoveCommand*>(cmd_iter.get());
                auto moveParam = movCmd->getMoveParam();
                auto speed = moveParam[0];
                auto t_WPnt = m_TargetProgramPtr->getWaypoint_byCommand(cmd_iter);
                if(firstMoveCmd){
                    if(flag_simFromCurrentPose){
                        m_InitPose = t_GroupPtr->getCurrentGroupPose(CordType::WCS);
                        _1stFrame = std::make_shared<GroupFrame>(m_InitPose);
                    }
                    firstMoveCmd = false;
                    skipframe = true;
                }

                _2ndFrame = std::make_shared<GroupFrame>(t_WPnt->getWPPoseData());
                if(flag_SwitchedTool){
                    path_Buffer = std::make_shared<GroupPath>(_1stFrame);
                    m_SimTracBuffer->Add(path_Buffer);
                    flag_SwitchedTool = false;
                    skipframe = true;
                }

                if(!skipframe){
                    if(movCmd->getMovePrec() == MovePrec::CNT){
                        if(!flag_BuildingRoundPath){
                            auto bl = moveParam[1];
                            auto vbl = moveParam[2];
                            path_Buffer = std::make_shared<GroupPath>(_1stFrame,
                                                                      _2ndFrame,
                                                                      speed,acc,
                                                                      bl,vbl);
                            flag_BuildingRoundPath = true;
                        }
                        else{
                            path_Buffer->addFrame(_2ndFrame);
                        }
                    }
                    else if(movCmd->getMovePrec() == MovePrec::FINE){
                        if(flag_BuildingRoundPath){
                            path_Buffer->addFrame(_2ndFrame);
                            path_Buffer->FinishPath(t_GroupPtr->getExtAxisSpeedLimits());
                            flag_BuildingRoundPath = false;
                        }
                        else{
                            path_Buffer = std::make_shared<GroupPath>(_1stFrame,
                                                                      _2ndFrame,
                                                                      speed,acc,
                                                                      t_GroupPtr->getExtAxisSpeedLimits());
                        }
                        flag_PathComplete = true;
                    }
                }

                if(flag_PathComplete){
                    m_SimTracBuffer->Add(path_Buffer);
                    flag_PathComplete = false;
                }
                skipframe = false;
                _1stFrame = _2ndFrame;
                break;
            }
            case CommandType::ChgTool:{
                auto toolCmd = static_cast<ToolCommand*>(cmd_iter.get());
                currentToolType = toolCmd->getToolType();
                t_GroupPtr->setCurrentToolType(currentToolType);
                if(_1stFrame != nullptr){
//                    path_Buffer = std::make_shared<GroupPath>(_1stFrame);
//                    m_GroupTracPtr->Add(path_Buffer);
                    flag_SwitchedTool = true;
                }
                break;
            }
            default:
                break;
            }
        }
        return true;
    }
    catch (KDL::Error &e) {
        throw Base::RuntimeError(e.Description());
    }
    return -3;
}

void TracSimulator::setTargetSimProgram(const RobotProg_sptr t_Program)
{
    m_TargetProgramPtr = t_Program;
}

const RobotProg_sptr &TracSimulator::getCurrentSimProgram() const
{
    return m_TargetProgramPtr;
}

const std::size_t TracSimulator::getMatchedCommandID(const double t_Time,
                                                     const std::size_t t_CmdID)
{
    if(t_CmdID >= m_MovCmdBuffer.size())
        return t_CmdID;
    auto t_CmmdPtr = m_MovCmdBuffer.at(t_CmdID);
    auto executorName = t_CmmdPtr->getOperatorName();
    m_GroupPtr = static_cast<Robot::MechanicGroup*>(m_DocPtr->getObject(executorName.c_str()));
    if(m_GroupPtr == nullptr)
        return t_CmdID;

    bool cmd_Finish = false;

    switch(t_CmmdPtr->getType()){
    case CommandType::SetMove:{
        updatePoseBuffer_ByTime(t_Time);
        m_GroupPtr->setGroupPose(m_PoseBuffer);
        auto t_cmd = static_cast<MoveCommand*>(t_CmmdPtr.get());
        cmd_Finish = moveCommandFinished(t_cmd);
        break;
    }
    case CommandType::OptTool:{
        auto t_cmd = static_cast<ToolCommand*>(t_CmmdPtr.get());
        m_GroupPtr->setCurrentToolActive(t_cmd->getToolStatus());
        cmd_Finish = true;
        break;
    }
    case CommandType::ChgTool:{
        auto t_cmd = static_cast<ToolCommand*>(t_CmmdPtr.get());
        m_GroupPtr->setCurrentToolType(t_cmd->getToolType());
        cmd_Finish = true;
        break;
    }
    default:
        cmd_Finish = true;
        break;
    }
    return cmd_Finish?t_CmdID+1:t_CmdID;
}

bool TracSimulator::moveCommandFinished(MoveCommand *t_cmmdPtr)
{
    if(m_GroupPtr == nullptr)
        return false;
    bool reached = true;
    auto movParam = t_cmmdPtr->getMoveParam();
    auto radius = movParam[1]*6;
    if(t_cmmdPtr->getMovePrec() == MovePrec::FINE)
        radius = 2.0;
    auto t_Pose = m_TargetProgramPtr->getWaypoint_byID(t_cmmdPtr->getMovPoseID());
    auto c_Pose = m_GroupPtr->getCurrentGroupPose(CordType::WCS);

    switch(t_Pose->getWPType()){
    case PoseType::CART:{
        reached &= DS_Utility::ifPlacementSame(c_Pose.Pose_Rbt1.getCartPose(),
                                               t_Pose->getWPCartPose_GP1(),
                                               radius);
        reached &= DS_Utility::ifPlacementSame(c_Pose.Pose_Rbt2.getCartPose(),
                                               t_Pose->getWPCartPose_GP2(),
                                               radius);
//        auto diffPose = c_Pose.inverse() * t_Pose->getWPCartPose_GP1();
//        auto c_dist = diffPose.getPosition().Length();
//        reached &= c_dist <= radius;
//        double y,p,r;
//        diffPose.getRotation().getYawPitchRoll(y,p,r);
//        reached &= y<0.5 && p<0.5 && r<0.5;
    }
        break;
    case PoseType::JNTS:
        break;
    }

    return reached;
}

void TracSimulator::resetSimulator(void)
{
    if(m_GroupPtr == nullptr)
        return;
    m_GroupPtr->setGroupPose(m_InitPose);
    m_GroupPtr->InteractiveTeach.setValue(true);
}


const double TracSimulator::getDuration(int n) const
{
    if(m_SimTracBuffer)
        return m_SimTracBuffer->getTracDuration();
    else
        return 0;
}

const RobotCommand_sptr TracSimulator::getSelectedCommandPtr(const size_t t_ID) const
{
    if(m_TargetProgramPtr == nullptr)
        return nullptr;
    return m_TargetProgramPtr->getCMD_byPosition(t_ID);
}

const RobotWaypoint_sptr TracSimulator::getSelectedCommandPose(const RobotCommand_sptr t_CmdPtr) const
{
    if(t_CmdPtr == nullptr || m_TargetProgramPtr == nullptr)
        return nullptr;
    return m_TargetProgramPtr->getWaypoint_byCommand(t_CmdPtr);
}

const size_t TracSimulator::getTotalCommandNumber() const
{
    if(m_TargetProgramPtr == nullptr)
        return 0;
    return m_TargetProgramPtr->getCmmdData().size();
}

const size_t TracSimulator::getMoveBufferCommandNum() const
{
    return m_MovCmdBuffer.size();
}

bool TracSimulator::executeSelectedCommand(const RobotCommand_sptr t_CmdPtr)
{
    if(t_CmdPtr == nullptr)
        return false;
    auto t_GroupPtr = static_cast<Robot::MechanicGroup*>(m_DocPtr->getObject(t_CmdPtr->getOperatorName().c_str()));
    if(t_GroupPtr == nullptr)
        return false;

    auto cmdName = m_TargetProgramPtr->generateCommandStr(t_CmdPtr);
    std::string msg = "\nExecuting Command: " + cmdName;
    Base::Console().Message(msg.c_str());

    bool result = false;
    switch(t_CmdPtr->getType()){
    case CommandType::SetMove:
        result = t_GroupPtr->setGroupPose(m_TargetProgramPtr->getWaypoint_byCommand(t_CmdPtr)->getWPPoseData());
        break;
    case CommandType::ChgTool:{
        auto t_ToolCmdPtr = static_cast<Robot::ToolCommand*>(t_CmdPtr.get());
        t_GroupPtr->setCurrentToolType(t_ToolCmdPtr->getToolType());
        result = true;
    }
        break;
    case CommandType::OptTool:{
        auto t_ToolCmdPtr = static_cast<Robot::ToolCommand*>(t_CmdPtr.get());
        t_GroupPtr->setCurrentToolActive(t_ToolCmdPtr->getToolStatus());
        result = true;
    }
    default:
        break;
    }

    msg = result?"...(DONE!)\n":"...(FAILED!)\n";
    Base::Console().Message(msg.c_str());

    return result;
}


bool TracSimulator::updatePoseBuffer_ByTime(double time)
{
    if(m_SimTracBuffer!=nullptr &&
       0 < time && time < m_SimTracBuffer->getTracDuration()){
        m_PoseBuffer = m_SimTracBuffer->getGroupPose(time);
        return true;
    }
    return false;
}

bool TracSimulator::isCommandIDValid(const size_t t_ID)
{
    return t_ID<m_TargetProgramPtr->getCmmdData().size();
}

bool TracSimulator::executeToTime(const double t_Time)
{

}

const double GroupTrac::getTracDuration() const
{
    return m_Duration;
}
