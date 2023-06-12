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
# include <Inventor/SoDB.h>
# include <Inventor/SoInput.h>
# include <Inventor/SbVec3f.h>
# include <Inventor/nodes/SoSeparator.h>
# include <Inventor/nodes/SoTransform.h>
# include <Inventor/nodes/SoSphere.h>
# include <Inventor/nodes/SoRotation.h>
# include <Inventor/actions/SoSearchAction.h>

# include <Inventor/nodes/SoBaseColor.h>
# include <Inventor/nodes/SoCoordinate3.h>
# include <Inventor/nodes/SoDrawStyle.h>
# include <Inventor/nodes/SoFaceSet.h>
# include <Inventor/nodes/SoLineSet.h>
# include <Inventor/nodes/SoMarkerSet.h>
# include <Inventor/nodes/SoShapeHints.h>
# include <QFile>
#endif

#include <Base/Console.h>
#include <Gui/Command.h>
#include <Gui/Control.h>
#include "ViewProviderTaskObject.h"
#include "TaskDlg_TaskManage.h"

using namespace RobotGui;

PROPERTY_SOURCE(RobotGui::ViewProviderTaskObject, Gui::ViewProviderGeometryObject)

ViewProviderTaskObject::ViewProviderTaskObject()
{

}

ViewProviderTaskObject::~ViewProviderTaskObject()
{

}

void ViewProviderTaskObject::attach(App::DocumentObject *pcObj)
{
    claimChildren();
//    ViewProviderRobotTrajectory::attach(pcObj);
}

void ViewProviderTaskObject::setDisplayMode(const char* ModeName)
{
//    ViewProviderRobotTrajectory::setDisplayMode( ModeName );
}

std::vector<std::string> ViewProviderTaskObject::getDisplayModes(void) const
{
    std::vector<std::string> StrList;
//    StrList.push_back("TracsOnly");
//    return ViewProviderRobotTrajectory::getDisplayModes();
    return StrList;
}

void ViewProviderTaskObject::updateData(const App::Property* prop)
{
    Robot::TaskObject* taskObj_Ptr = static_cast<Robot::TaskObject*>(pcObject);
    if (prop == &taskObj_Ptr->ActionList) {
        claimChildren();
    }
    else if(prop == &taskObj_Ptr->refreshTracData){
//        if(taskObj_Ptr->refreshTracData.getValue()){
////            pcWaypoints_Gp1->point.deleteValues(0);
////            pcWaypoints_Gp2->point.deleteValues(0);
////            auto total_wpNum = taskObj_Ptr->getTaskTracPointNumber();
////            pcWaypoints_Gp1->point.setNum(total_wpNum);
////            pcWaypoints_Gp2->point.setNum(total_wpNum);
////            const auto& programSptr = taskObj_Ptr->getProgramPtr();
////            for(uint i = 0; i< programSptr->getTotalPoseNumber(); i++){
////                auto t_Pose = programSptr->getWaypoint_byPosition(i)->getWPCartPose_GP1();
////                Base::Vector3d pos = t_Pose.getPosition();
////                pcWaypoints_Gp1->point.set1Value(i,pos.x,pos.y,pos.z);
////            }
////            pcPathLine_Gp1->numVertices.set1Value(0, total_wpNum);
//            taskObj_Ptr->refreshTracData.setValue(false);
//        }
    }
}

bool ViewProviderTaskObject::doubleClicked()
{
    std::string Msg("Edit ");
    Msg += this->pcObject->Label.getValue();
    try {
      Gui::Command::openCommand(Msg.c_str());
      Gui::Command::doCommand(Gui::Command::Gui,
                              "Gui.ActiveDocument.setEdit('%s',%d)",
                              this->pcObject->getNameInDocument(),
                              Gui::ViewProvider::EditMode::Default);
      return true;
    }
    catch (const Base::Exception &e) {
      Base::Console().Error("%s\n", e.what());
      return false;
    }
}

std::vector<App::DocumentObject *> ViewProviderTaskObject::claimChildren() const
{
    Robot::TaskObject* taskObj_Ptr = static_cast<Robot::TaskObject*>(pcObject);
    if(taskObj_Ptr!= nullptr)
        return taskObj_Ptr->getActionList();
    return std::vector<App::DocumentObject *>();
}

bool ViewProviderTaskObject::setEdit(int ModNum)
{
    Robot::TaskObject* taskObj_Ptr = static_cast<Robot::TaskObject*>(pcObject);
    auto tskManageDlg_Ptr = std::make_shared<TaskDlg_TaskManage>(taskObj_Ptr);
    if(tskManageDlg_Ptr == nullptr)
        return false;
    Gui::Control().show_TaskManageDialog(tskManageDlg_Ptr);
}

bool ViewProviderTaskObject::onDelete(const std::vector<string> &subNames)
{
    Robot::TaskObject* taskObj_Ptr = static_cast<Robot::TaskObject*>(pcObject);
    taskObj_Ptr->onDelete();
    return ViewProviderGeometryObject::onDelete(subNames);
}

