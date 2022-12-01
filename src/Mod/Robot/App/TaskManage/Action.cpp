/***************************************************************************
 *   Copyright (c) 2008 Jürgen Riegel (juergen.riegel@web.de)              *
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
#include "Action.h"

//#include <App/DocumentObjectPy.h>

using namespace Robot;

TYPESYSTEM_SOURCE(Robot::Action, Base::Persistence)

Action::Action()
{
}


Action::~Action()
{
}

void Action::Save(Base::Writer &writer) const
{
    writer.Stream()<<"TaskName=\""<<_taskName<<"\" ";
    writer.Stream()<<"OperName=\""<<_operatorName<<"\" ";
    writer.Stream()<<"Duration=\""<<_duration<<"\" ";
}

void Action::Restore(Base::XMLReader &reader)
{
    _taskName = std::string(reader.getAttribute("TaskName"));
    _operatorName = std::string(reader.getAttribute("OperName"));
    _duration = reader.getAttributeAsFloat("Duration");
}
