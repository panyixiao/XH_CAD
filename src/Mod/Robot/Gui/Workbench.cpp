/***************************************************************************
 *   Copyright (c) 2008 Werner Mayer <werner.wm.mayer@gmx.de>              *
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


#include "PreCompiled.h"

#ifndef _PreComp_
# include <qobject.h>
# include <QDir>
# include <QFileInfo>
# include <QMessageBox>
#endif

#include "Workbench.h"
#include <App/Application.h>
#include <Gui/ToolBarManager.h>
#include <Gui/MenuManager.h>
#include <Gui/MainWindow.h>
#include <Gui/Control.h>
#include <Gui/WaitCursor.h>
#include <Gui/TaskView/TaskView.h>
#include <Gui/TaskView/TaskWatcher.h>

//#include "TaskWatcher.h"


using namespace RobotGui;

#if 0 // needed for Qt's lupdate utility
    qApp->translate("Workbench", "Robot");
    qApp->translate("Workbench", "Insert Robots");
    qApp->translate("Workbench", "&Robot");
    qApp->translate("Workbench", "Export trajectory");
    qApp->translate("Gui::TaskView::TaskWatcherCommands", "Trajectory tools");
    qApp->translate("Gui::TaskView::TaskWatcherCommands", "Robot tools");
    qApp->translate("Gui::TaskView::TaskWatcherCommands", "Insert Robot");
#endif

/// @namespace RobotGui @class Workbench
TYPESYSTEM_SOURCE(RobotGui::Workbench, Gui::StdWorkbench)

Workbench::Workbench()
{
}

Workbench::~Workbench()
{
}

void Workbench::activated()
{
    Gui::Workbench::activated();
    const char* ModelOperation[] = {
        "Import_CAD_File",
        "ImportTool"
        };

    const char* RobotAndTrac[] = {
        "Robot_InsertWaypoint",
        "Robot_InsertWaypointPreselect",
        0};

    const char* RobotOperation[] = {
//        "Robot_AddToolShape",
        "Robot_SetHomePos",
        "Robot_RestoreHomePos",
        0};

    const char* TracSingle[] = {
        "Robot_TrajectoryDressUp",
        0};

    const char* TracMore[] = {
        "Robot_TrajectoryCompound",
        0};

    std::vector<Gui::TaskView::TaskWatcher*> Watcher;

    Watcher.push_back(new Gui::TaskView::TaskWatcherCommands(
        "SELECT Robot::TrajectoryObject COUNT 1"
        "SELECT Robot::RobotObject COUNT 1",
        RobotAndTrac,
        "Trajectory tools",
        "Robot_InsertWaypoint"
    ));

//    Watcher.push_back(new TaskWatcherRobot);

    Watcher.push_back(new Gui::TaskView::TaskWatcherCommands(
        "SELECT Robot::RobotObject COUNT 1",
        RobotOperation,
        "Robot tools",
        "Robot_CreateRobot"
    ));

    Watcher.push_back(new Gui::TaskView::TaskWatcherCommands(
        "SELECT Robot::TrajectoryObject COUNT 1",
        TracSingle,
        "Trajectory tools",
        "Robot_CreateRobot"
    ));

    Watcher.push_back(new Gui::TaskView::TaskWatcherCommands(
        "SELECT Robot::TrajectoryObject COUNT 2..",
        TracMore,
        "Trajectory tools",
        "Robot_CreateRobot"
    ));

    
    addTaskWatcher(Watcher);
//    Gui::Control().showTaskView();
}


void Workbench::deactivated()
{
    Gui::Workbench::deactivated();
    removeTaskWatcher();
}


Gui::ToolBarItem* Workbench::setupToolBars() const
{
    Gui::ToolBarItem* root = Gui::RBT_Workbench::setupToolBars();

    Gui::ToolBarItem* model = new Gui::ToolBarItem(root);
    model->setCommand("model");
    *model << "Import_CAD_File";
//    *model << "ImportTool";

    Gui::ToolBarItem* robot = new Gui::ToolBarItem(root);
    robot->setCommand("Robot");
    *robot << "Robot_InsertMechanicRobot"
           << "Robot_InsertPositioner"
           << "Robot_InsertExtAxisDevice"
           << "Separator";

    Gui::ToolBarItem* tool = new Gui::ToolBarItem(root);
    tool->setCommand("Tool");
    *tool << "Command_InsertWeldTorch";
    *tool << "Command_InsertLaserScanner";
    *tool << "Separator";

    Gui::ToolBarItem* task = new Gui::ToolBarItem(root);
    task->setCommand("TaskManage");
    *task <<"Command_CreateNewTask"
         << "Separator";

    return root;
}

Gui::MenuItem* Workbench::setupMenuBar() const
{
    Gui::MenuItem* root = Gui::RBT_Workbench::setupMenuBar();
    Gui::MenuItem* item = root->findItem("&Windows");
    Gui::MenuItem* robot = new Gui::MenuItem;
    root->insertItem( item, robot );

    // analyze
    Gui::MenuItem* insertRobots = new Gui::MenuItem;
    insertRobots->setCommand("Insert Robots");
    *insertRobots << "Robot_InsertMechanicRobot"
                  << "Separator"
//                  << "Robot_AddToolShape"
                  ;
    return root;
}
