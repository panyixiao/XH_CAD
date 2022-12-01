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


#include "Mod/Robot/Gui/PreCompiled.h"

#ifndef _PreComp_
#endif

#include "ViewProviderEdgebasedTracObject.h"
#include <Gui/Control.h>
//#include <Mod/Robot/Gui/TaskDlgEdge2Trac.h>
#include "Mod/Robot/App/Trac/EdgebasedTracObject.h"
#include "Mod/Robot/Gui/TaskManage/TaskDlg_TaskManage.h"

using namespace Gui;
using namespace RobotGui;

PROPERTY_SOURCE(RobotGui::ViewProviderEdgebasedTracObject, RobotGui::ViewProviderRobotTrajectory)

bool ViewProviderEdgebasedTracObject::doubleClicked(void)
{
    return setEdit(0);
}

void ViewProviderEdgebasedTracObject::updateData(const App::Property *prop)
{
    ViewProviderRobotTrajectory::updateData(prop);
}


bool ViewProviderEdgebasedTracObject::setEdit(int ModNum)
{
    auto tskObjectPtr = dynamic_cast<Robot::EdgebasedTracObject*>(getObject());
    auto tskManageDlg_Ptr = std::make_shared<TaskDlg_TaskManage>(tskObjectPtr);
    if(tskManageDlg_Ptr == nullptr)
        return false;
    Gui::Control().show_TaskManageDialog(tskManageDlg_Ptr);

    return true;
}

void ViewProviderEdgebasedTracObject::unsetEdit(int ModNum)
{


}
