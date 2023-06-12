// Created By Yixiao 2022-06-22

#include "Mod/Robot/App/PreCompiled.h"

#ifndef _PreComp_
#endif

#include <Base/Writer.h>
#include <Base/Reader.h>
#include <Base/Exception.h>

#include "CommandBase.h"

using namespace Robot;

TYPESYSTEM_SOURCE(Robot::CommandBase , Base::Persistence);

CommandBase::CommandBase(){}

CommandBase::CommandBase(const CommandBase &rhs)
{
    m_CmdType = rhs.m_CmdType;
}

CommandBase::~CommandBase(){}

void CommandBase::setCommandType(const CommandType &t_Type)
{
    m_CmdType = t_Type;
}

unsigned int CommandBase::getMemSize() const
{
    return 0;
}

void CommandBase::Save(Base::Writer &writer) const{
    writer.Stream() << writer.ind() << "<Command ";
    writer.Stream() << "CmdType=\""<<std::to_string((uint)m_CmdType)<<"\" ";
    Base::Persistence::Save(writer);
}

void CommandBase::Restore(Base::XMLReader &reader) {
    Base::Persistence::Restore(reader);
}

const CommandType &CommandBase::getType() const
{
    return m_CmdType;
}





 
