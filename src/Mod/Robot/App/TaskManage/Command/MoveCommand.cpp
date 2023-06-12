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

#include "MoveCommand.h"

using namespace Robot;

TYPESYSTEM_SOURCE(Robot::MoveCommand , Robot::CommandBase);

MoveCommand::MoveCommand()
{
    m_CmdType = CommandType::SetMove;
}

MoveCommand::MoveCommand(const MoveCommand &rhs)
{
//    _operatorName = rhs._operatorName;
    m_CmdType = rhs.m_CmdType;
    m_MovType = rhs.m_MovType;
    m_MovPrec = rhs.m_MovPrec;
    m_PoseID = rhs.m_PoseID;
    m_Vel = rhs.m_Vel;
    m_BL = rhs.m_BL;
    m_VBL = rhs.m_VBL;
}

MoveCommand::~MoveCommand()
{
}

MoveCommand::MoveCommand(const MoveType &t_Type,
                         const MovePrec &t_Prec,
                         const size_t pntID,
                         const float t_V,
                         const float t_BL,
                         const float t_VBL)
{
//    _taskName = taskName;
//    _operatorName = executor;
    m_CmdType = CommandType::SetMove;
    m_MovType = t_Type;
    m_MovPrec = t_Prec;
    m_PoseID = pntID;
    m_Vel = t_V;

    if(m_MovPrec == MovePrec::CNT){
        m_BL = t_BL;
        m_VBL = t_VBL;
    }else{
        m_BL = 0;
        m_VBL = 0;
    }
}

void MoveCommand::Save(Base::Writer &writer) const
{
    CommandBase::Save(writer);
    writer.Stream() << "PoseID=\"" <<std::to_string(m_PoseID)<<"\" ";
    writer.Stream() << "M_Type=\"" <<std::to_string((uint)m_MovType)<<"\" ";
    writer.Stream() << "Prec=\"" <<std::to_string((uint)m_MovPrec)<<"\" ";
    writer.Stream() << "V_Type=\"" <<std::to_string((uint)m_SpeedType)<<"\" ";
    writer.Stream() << "SpeedVal=\"" << std::to_string(m_Vel)<<"\" ";
    writer.Stream() << "BL=\"" << std::to_string(m_BL)<<"\" ";
    writer.Stream() << "VBL=\"" << std::to_string(m_VBL)<<"\"";
    writer.Stream()<<"/>"<<std::endl;
}

void MoveCommand::Restore(Base::XMLReader &reader)
{
    CommandBase::Restore(reader);
    m_PoseID = reader.getAttributeAsUnsigned("PoseID");
    m_MovType = (MoveType)reader.getAttributeAsUnsigned("M_Type");
    m_MovPrec = (MovePrec)reader.getAttributeAsUnsigned("Prec");
    m_SpeedType = (SpeedType)reader.getAttributeAsUnsigned("V_Type");
    m_Vel = reader.getAttributeAsFloat("SpeedVal");
    m_BL = reader.getAttributeAsFloat("BL");
    m_VBL = reader.getAttributeAsFloat("VBL");
}





 
