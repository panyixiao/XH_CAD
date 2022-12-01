// Created By Yixiao 2022-06-22

#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Exception.h>

#include "RobotCommand.h"

using namespace Robot;

TYPESYSTEM_SOURCE(Robot::RobotCommand , Robot::Action);

RobotCommand::RobotCommand(){}

RobotCommand::RobotCommand(const RobotCommand &rhs)
{
    m_CmdType = rhs.m_CmdType;
}

RobotCommand::~RobotCommand(){}

void RobotCommand::setCommandType(const CommandType &t_Type)
{
    m_CmdType = t_Type;
}

unsigned int RobotCommand::getMemSize() const
{
    return 0;
}

void RobotCommand::Save(Base::Writer &writer) const{
    writer.Stream() << writer.ind() << "<Command ";
    writer.Stream() << "CmdType=\""<<std::to_string((uint)m_CmdType)<<"\" ";
    Action::Save(writer);
}

void RobotCommand::Restore(Base::XMLReader &reader) {
    Action::Restore(reader);
}

const CommandType &RobotCommand::getType() const
{
    return m_CmdType;
}





 
