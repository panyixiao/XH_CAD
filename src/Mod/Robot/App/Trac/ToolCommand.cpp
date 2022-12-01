// Created By Yixiao 2022-06-22

#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Exception.h>

#include "ToolCommand.h"

using namespace Robot;

TYPESYSTEM_SOURCE(Robot::ToolCommand , Robot::CoordCommand);

ToolCommand::ToolCommand()
{
    m_CmdType = CommandType::ChgTool;
    m_CoordType = CordType::TCS;
    m_ToolType = ToolType::Flan;
}

ToolCommand::ToolCommand(const ToolCommand &rhs)
{
    _operatorName = rhs._operatorName;
    m_CmdType = rhs.m_CmdType;
    m_CoordType = rhs.m_CoordType;
    m_ToolType = rhs.m_ToolType;
    m_status = rhs.m_status;
}

ToolCommand::ToolCommand(const std::string taskName, const std::string executorName,
                         const ToolType &t_Type, const uint coordID)
{
    _taskName = taskName;
    _operatorName = executorName;
    m_CmdType = CommandType::ChgTool;
    m_CoordType = CordType::TCS;
    m_CoordID = coordID;
    m_ToolType = t_Type;
    m_status = false;
}

ToolCommand::ToolCommand(const std::string taskName, const std::string executorName,
                         const ToolType &t_Type,
                         const bool status)
{
    _taskName = taskName;
    _operatorName = executorName;
    m_CmdType = CommandType::OptTool;
    m_CoordType = CordType::TCS;
    m_ToolType = t_Type;
    m_status = status;
}

ToolCommand::~ToolCommand()
{
}

// TODO : Tool Operation Command Save is not okay
void ToolCommand::Save(Base::Writer &writer) const
{
    RobotCommand::Save(writer);
    writer.Stream() << "ToolType=\""<<std::to_string((uint)m_ToolType)<<"\" ";
    writer.Stream() <<"Status=\""<< std::to_string((uint)m_status)<<"\" ";
    writer.Stream()<<"/>"<< std::endl;
}

void ToolCommand::Restore(Base::XMLReader &reader)
{
    RobotCommand::Restore(reader);
    m_ToolType = (ToolType)reader.getAttributeAsUnsigned("ToolType");
    m_status = (bool)reader.getAttributeAsUnsigned("Status");
}




 
