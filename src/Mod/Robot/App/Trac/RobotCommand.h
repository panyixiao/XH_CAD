// Created By Yixiao 2022-06-21

#ifndef ROBOT_COMMAND_H
#define ROBOT_COMMAND_H

#include <memory>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <Base/Persistence.h>
#include <Mod/Robot/App/TaskManage/Action.h>
#include <vector>

/// 2. Cmd Type:
///    Robot Move  Command: Move action + Pose
///    Param Setup Command: Set Velocity/Acc/Tool ID etc
///    Device Opt  Command: To change I/O status

namespace Robot
{

enum class CommandType{
    SetMove = 0,
    SetCord,
    ChgTool,
    OptTool,
    SetComm,
    SetIOport,
    Customize
};

class RobotExport RobotCommand : public Robot::Action
{
    TYPESYSTEM_HEADER();
public:
    RobotCommand();
    RobotCommand(const RobotCommand&rhs);
    ~RobotCommand();
    void setCommandType(const CommandType& t_Type);
    const CommandType& getType() const;
    virtual unsigned int getMemSize (void) const;
    virtual void Save (Base::Writer &writer) const;
    virtual void Restore(Base::XMLReader &reader);
//    const std::size_t getCommandID() const{
//        return boost::uuids::hash_value(m_CmdID);
//    }

protected:
//    boost::uuids::uuid m_CmdID;
    CommandType m_CmdType;
};

} //namespace Part


#endif // PART_TOPOSHAPE_H
