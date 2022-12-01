// Created By Yixiao 2022-06-22

#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Exception.h>

#include "CoordCommand.h"

using namespace Robot;

TYPESYSTEM_SOURCE(Robot::CoordCommand , Robot::RobotCommand);

CoordCommand::CoordCommand()
{
    m_CmdType = CommandType::SetCord;
}

CoordCommand::CoordCommand(const CoordCommand &rhs)
{
    _operatorName = rhs._operatorName;
    m_CmdType = rhs.m_CmdType;
    m_CoordType = rhs.m_CoordType;
    m_CoordID = rhs.m_CoordID;
}

CoordCommand::~CoordCommand()
{
}

CoordCommand::CoordCommand(const std::string& taskName,
                           const std::string& executorName,
                           const CordType &t_Coord,
                           const uint t_ID)
{
    _taskName = taskName;
    _operatorName = executorName;
    m_CmdType = CommandType::SetCord;
    m_CoordType = t_Coord;
    m_CoordID = t_ID;
}

void CoordCommand::Save(Base::Writer &writer) const
{
    RobotCommand::Save(writer);
    writer.Stream() << "CordType=\""<<std::to_string((uint)m_CoordType)<<"\" ";
    writer.Stream() << "CordID=\""<<std::to_string(m_CoordID)<<"\" ";
    writer.Stream()<<"/>"<<std::endl;
}

void CoordCommand::Restore(Base::XMLReader &reader)
{
    RobotCommand::Restore(reader);
    m_CoordType = (CordType)(reader.getAttributeAsUnsigned("CordType"));
    m_CoordID = reader.getAttributeAsUnsigned("CordID");
}




 
