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
# include <Inventor/draggers/SoJackDragger.h>
# include <Inventor/VRMLnodes/SoVRMLTransform.h>
# include <Inventor/nodes/SoBaseColor.h>
# include <Inventor/nodes/SoCoordinate3.h>
# include <Inventor/nodes/SoDrawStyle.h>
# include <Inventor/nodes/SoFaceSet.h>
# include <Inventor/nodes/SoLineSet.h>
# include <Inventor/nodes/SoMarkerSet.h>
# include <Inventor/nodes/SoShapeHints.h>
# include <QFile>
#endif

#include "ViewProviderRobotTrajectory.h"

//#include <Mod/Robot/App/TrajectoryObject.h>
#include <Mod/Robot/App/Trac/RobotTracObject.h>
//#include <Mod/Robot/App/Trajectory.h>
#include <Mod/Robot/Gui/TaskManage/TaskDlg_TaskManage.h>

#include <App/Document.h>
#include <Base/FileInfo.h>
#include <Base/Stream.h>
#include <Base/Console.h>
#include <Gui/Control.h>
#include <sstream>
using namespace Gui;
using namespace RobotGui;
using namespace Robot;

PROPERTY_SOURCE(RobotGui::ViewProviderRobotTrajectory, Gui::ViewProviderGeometryObject)

ViewProviderRobotTrajectory::ViewProviderRobotTrajectory()
{
    
	pcTrajectoryRoot = new Gui::SoFCSelection();
    pcTrajectoryRoot->highlightMode = Gui::SoFCSelection::OFF;
    pcTrajectoryRoot->selectionMode = Gui::SoFCSelection::SEL_OFF;
    pcTrajectoryRoot->ref();

    pcWaypoints_Gp1 = new SoCoordinate3();
    pcWaypoints_Gp1->ref();
    pcPathLine_Gp1 = new SoLineSet;
    pcPathLine_Gp1->ref();

    pcWaypoints_Gp2 = new SoCoordinate3();
    pcWaypoints_Gp2->ref();
    pcPathLine_Gp2 = new SoLineSet;
    pcPathLine_Gp2->ref();

    pcDrawStyle = new SoDrawStyle();
    pcDrawStyle->ref();
    pcDrawStyle->style = SoDrawStyle::LINES;
    pcDrawStyle->lineWidth = 2.5;
}

ViewProviderRobotTrajectory::~ViewProviderRobotTrajectory()
{
    pcTrajectoryRoot->unref();
    pcDrawStyle->unref();
    pcWaypoints_Gp1->unref();
    pcPathLine_Gp1->unref();
    pcWaypoints_Gp2->unref();
    pcPathLine_Gp2->unref();
}

void ViewProviderRobotTrajectory::attach(App::DocumentObject *pcObj)
{
    ViewProviderDocumentObject::attach(pcObj);

    SoMarkerSet* marker = new SoMarkerSet;
    marker->markerIndex=SoMarkerSet::CROSS_5_5;

    // Draw trajectory lines
    SoSeparator* gp1_Path = new SoSeparator;
    SoBaseColor * gp1_lineColor = new SoBaseColor;
    gp1_lineColor->rgb.setValue( 1.0f, 0.5f, 0.0f );
    gp1_Path->addChild(gp1_lineColor);
    gp1_Path->addChild(pcWaypoints_Gp1);
    gp1_Path->addChild(pcPathLine_Gp1);
    SoBaseColor * gp1_MarkColor = new SoBaseColor;
    gp1_MarkColor->rgb.setValue( 1.0f, 0.5f, 0.0f );
    gp1_Path->addChild(gp1_MarkColor);
    gp1_Path->addChild(marker);

    SoSeparator* gp2_Path = new SoSeparator;
    SoBaseColor * gp2_lineColor = new SoBaseColor;
    gp2_lineColor->rgb.setValue( 0.0f, 0.5f, 1.0f );
    gp2_Path->addChild(gp2_lineColor);
    gp2_Path->addChild(pcWaypoints_Gp2);
    gp2_Path->addChild(pcPathLine_Gp2);
    SoBaseColor * gp2_MarkColor = new SoBaseColor;
    gp2_MarkColor->rgb.setValue( 0.0f, 0.5f, 1.0f );
    gp2_Path->addChild(gp2_MarkColor);
    gp2_Path->addChild(marker);


    pcTrajectoryRoot->addChild(gp1_Path);
    pcTrajectoryRoot->addChild(gp2_Path);

    addDisplayMaskMode(pcTrajectoryRoot, "Visible");
    pcTrajectoryRoot->objectName = pcObj->getNameInDocument();
    pcTrajectoryRoot->documentName = pcObj->getDocument()->getName();
    pcTrajectoryRoot->subElementName = "Main";

}

void ViewProviderRobotTrajectory::setDisplayMode(const char* ModeName)
{
    if ( strcmp("Visible",ModeName)==0 )
        setDisplayMaskMode("Visible");
    ViewProviderGeometryObject::setDisplayMode( ModeName );
}

std::vector<std::string> ViewProviderRobotTrajectory::getDisplayModes(void) const
{
    std::vector<std::string> StrList;
    StrList.push_back("Visible");
    return StrList;
}

void ViewProviderRobotTrajectory::updateData(const App::Property* prop)
{
    Robot::RobotTracObject* pcTracObj = static_cast<Robot::RobotTracObject*>(pcObject);
    if (prop == &pcTracObj->WaypointNumber) {
        updateTracDisplay();
    }
    else if (prop == &pcTracObj->TracOrigin) {
        updateTracDisplay();
    }

}

bool ViewProviderRobotTrajectory::setEdit(int ModNum)
{
    Robot::RobotTracObject* pcTracObj = static_cast<Robot::RobotTracObject*>(pcObject);
    auto t_dlgPtr = std::make_shared<TaskDlg_TaskManage>(pcTracObj);
    if(t_dlgPtr!=nullptr)
        Gui::Control().show_TaskManageDialog(t_dlgPtr);
}

void ViewProviderRobotTrajectory::updateTracDisplay()
{
    Robot::RobotTracObject* pcTracObj = static_cast<Robot::RobotTracObject*>(pcObject);
    if(pcTracObj->getRobotProgramSptr() == nullptr)
        return;
    auto waypoints = pcTracObj->getWaypointData();
    auto waypoitNum = waypoints.size();
    pcWaypoints_Gp1->point.deleteValues(0);
    pcWaypoints_Gp1->point.setNum(waypoitNum);
    pcWaypoints_Gp2->point.deleteValues(0);
    pcWaypoints_Gp2->point.setNum(waypoitNum);

    Base::Placement pose_buffer;
    for(unsigned int i=0;i<waypoitNum;++i){
        pose_buffer = waypoints.at(i)->getWPCartPose_GP1();
        pcWaypoints_Gp1->point.set1Value(i,
                                         pose_buffer.getPosition().x,
                                         pose_buffer.getPosition().y,
                                         pose_buffer.getPosition().z);

        pose_buffer = waypoints.at(i)->getWPCartPose_GP2();
        pcWaypoints_Gp2->point.set1Value(i,
                                         pose_buffer.getPosition().x,
                                         pose_buffer.getPosition().y,
                                         pose_buffer.getPosition().z);
    }
    pcPathLine_Gp1->numVertices.set1Value(0, waypoitNum);
    pcPathLine_Gp2->numVertices.set1Value(0, waypoitNum);
}

