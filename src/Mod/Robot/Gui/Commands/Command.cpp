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
# include <QMessageBox>
#endif

#include <App/Application.h>
#include <Gui/Application.h>
#include <Gui/MainWindow.h>
#include <Gui/Command.h>
#include <Gui/FileDialog.h>
#include <Gui/Selection.h>
#include <Gui/SelectionFilter.h>
#include <Gui/Document.h>
#include <Gui/Control.h>

//#include <Mod/Robot/App/RobotObject.h>
#include <Mod/Robot/App/Mechanics/Robot6AxisObject.h>
//#include <Mod/Robot/App/TrajectoryObject.h>

//#include "Mod/Robot/Gui/TrajectorySimulate.h"
//#include "Mod/Robot/Gui/TaskDlgSimulate.h"

using namespace std;
//using namespace RobotGui;


//// #####################################################################################################
//DEF_STD_CMD_A(CmdRobotConstraintAxle);

//CmdRobotConstraintAxle::CmdRobotConstraintAxle()
//	:Command("Robot_Create")
//{
//    sAppModule      = "Robot";
//    sGroup          = QT_TR_NOOP("Robot");
//    sMenuText       = QT_TR_NOOP("Place robot...");
//    sToolTipText    = QT_TR_NOOP("Place a robot (experimental!)");
//    sWhatsThis      = "Robot_Create";
//    sStatusTip      = sToolTipText;
//    sPixmap         = "Robot_CreateRobot";
//}

//void CmdRobotConstraintAxle::activated(int iMsg)
//{
//    std::string FeatName = getUniqueObjectName("Robot");
//    openCommand("Place robot");
//    doCommand(Doc,"App.activeDocument().addObject(\"Robot::Robot6AxisObject\",\"%s\")",FeatName.c_str());
//    updateActive();
//    commitCommand();
//}

//bool CmdRobotConstraintAxle::isActive(void)
//{
//    return hasActiveDocument();
//}


//// #####################################################################################################

//DEF_STD_CMD_A(CmdRobotSimulate);

//CmdRobotSimulate::CmdRobotSimulate()
//	:Command("Robot_Simulate")
//{
//    sAppModule      = "Robot";
//    sGroup          = QT_TR_NOOP("Simulate");
//    sMenuText       = QT_TR_NOOP("Simulate a trajectory");
//    sToolTipText    = QT_TR_NOOP("Run a simulation on a trajectory");
//    sWhatsThis      = "Robot_Simulate";
//    sStatusTip      = sToolTipText;
//    sPixmap         = "Robot_Simulate";
//}


//void CmdRobotSimulate::activated(int iMsg)
//{
//#if 1
//    const char * SelFilter = "SELECT Robot::Robot6AxisObject  \n"
//                             "SELECT Robot::TrajectoryObject  ";

//    Gui::SelectionFilter filter(SelFilter);
//    Robot::Robot6AxisObject *pcRobotObject;
//    Robot::TrajectoryObject *pcTrajectoryObject;

//    if (filter.match()) {
//        pcRobotObject = static_cast<Robot::Robot6AxisObject*>(filter.Result[0][0].getObject());
//        pcTrajectoryObject = static_cast<Robot::TrajectoryObject*>(filter.Result[1][0].getObject());;
//    }
//    else {
//        QMessageBox::warning(Gui::getMainWindow(), QObject::tr("Wrong selection"),
//            QObject::tr("Select one Robot and one Trajectory object."));
//        return;
//    }

//    if(pcTrajectoryObject->Trajectory.getValue().getSize() < 2){
//        QMessageBox::warning(Gui::getMainWindow(), QObject::tr("Trajectory not valid"),
//            QObject::tr("You need at least two way points in a trajectory to simulate."));
//        return;
//    }

//    Gui::TaskView::TaskDialog* dlg = new TaskDlgSimulate(pcRobotObject,pcTrajectoryObject);
//    Gui::Control().showDialog(dlg);
     
//#else

 
//   const char * SelFilter =
//       "SELECT Robot::Robot6AxisObject  \n"
//       "SELECT Robot::TrajectoryObject  ";

//    Gui::SelectionFilter filter(SelFilter);
//    Robot::Robot6AxisObject *pcRobotObject;
//    Robot::TrajectoryObject *pcTrajectoryObject;

//    if(filter.match()){
//        pcRobotObject = dynamic_cast<Robot::Robot6AxisObject*>(filter.Result[0][0].getObject());
//        pcTrajectoryObject = dynamic_cast<Robot::TrajectoryObject*>(filter.Result[1][0].getObject());;
//    }else{
//        QMessageBox::warning(Gui::getMainWindow(), QObject::tr("Wrong selection"),
//            QObject::tr("Select one Robot and one Trajectory object."));
//    }

//    RobotGui::TrajectorySimulate dlg(pcRobotObject,pcTrajectoryObject,Gui::getMainWindow());
//    dlg.exec();
//#endif
//}

//bool CmdRobotSimulate::isActive(void)
//{
//    return (hasActiveDocument() && !Gui::Control().activeDialog());
//}



// #####################################################################################################



void CreateRobotCommands(void)
{
    Gui::CommandManager &rcCmdMgr = Gui::Application::Instance->commandManager();

//    rcCmdMgr.addCommand(new CmdRobotConstraintAxle());
//    rcCmdMgr.addCommand(new CmdRobotSimulate());
 }
